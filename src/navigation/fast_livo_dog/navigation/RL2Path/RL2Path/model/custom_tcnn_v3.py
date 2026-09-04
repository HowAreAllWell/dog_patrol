import math
import os
import random
import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F

import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# ---------------- Utils ----------------
SEED1 = 3047
def set_seed(seed):
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)

# ---------------- LayerNorm over channels for NCHW ----------------
class ChannelLayerNorm(nn.Module):
    """
    对输入 (N, C, H, W) 做 LayerNorm，但只在通道维进行归一化。
    """
    def __init__(self, num_channels, eps=1e-5, affine=True):
        super().__init__()
        self.ln = nn.LayerNorm(normalized_shape=num_channels, eps=eps, elementwise_affine=affine)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (N, C, H, W)
        x = x.permute(0, 2, 3, 1)
        x = self.ln(x)
        x = x.permute(0, 3, 1, 2)
        return x

# ---------------- ResNet bits ----------------
def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    return nn.Conv2d(
        in_planes, out_planes, kernel_size=3, stride=stride,
        padding=dilation, groups=groups, bias=True, dilation=dilation
    )

def conv1x1(in_planes, out_planes, stride=1):
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=True)

class Bottleneck(nn.Module):
    expansion = 2
    def __init__(self, inplanes, planes, stride=1, downsample=None,
                 groups=1, base_width=64, dilation=1, norm_layer=None):
        super().__init__()
        width = int(planes * (base_width / 64.)) * groups
        self.conv1 = conv1x1(inplanes, width, stride=1)
        self.ln1   = ChannelLayerNorm(width)
        self.conv2 = conv3x3(width, width, stride=stride, groups=groups, dilation=dilation)
        self.ln2   = ChannelLayerNorm(width)
        self.conv3 = conv1x1(width, planes * self.expansion, stride=1)
        self.ln3   = ChannelLayerNorm(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x
        out = self.conv1(x); out = self.ln1(out); out = self.relu(out)
        out = self.conv2(out); out = self.ln2(out); out = self.relu(out)
        out = self.conv3(out); out = self.ln3(out)
        if self.downsample is not None:
            identity = self.downsample(x)
        out += identity
        out = self.relu(out)
        return out

# ---------------- FiLM 1D (vector-wise, identity-safe) ----------------
class FiLM1d(nn.Module):
    """
    对向量特征 z: [N, D] 做 FiLM: y = (1 + gamma) ⊙ z + beta
    - gamma/beta 由 cond 生成，线性层零初始化，tanh 限幅并乘小尺度
    """
    def __init__(self, feat_dim: int, cond_dim: int, gamma_scale: float = 0.1, beta_scale: float = 0.1):
        super().__init__()
        self.feat_dim = feat_dim
        self.proj = nn.Linear(cond_dim, 2 * feat_dim)
        nn.init.zeros_(self.proj.weight)
        nn.init.zeros_(self.proj.bias)
        self.gamma_scale = gamma_scale
        self.beta_scale = beta_scale

    def forward(self, z: torch.Tensor, cond_emb: torch.Tensor) -> torch.Tensor:
        gb = self.proj(cond_emb)                # [N, 2D]
        gamma, beta = gb.chunk(2, dim=1)        # [N, D], [N, D]
        gamma = self.gamma_scale * torch.tanh(gamma)
        beta  = self.beta_scale  * torch.tanh(beta)
        return z * (1 + gamma) + beta

# ---------------- Late-fusion head ----------------
class LateFusionHead(nn.Module):
    """
    将 trunk 向量特征 z 与条件嵌入 c 在“后连接”处融合。
    mode:
      - 'film'  : 1D-FiLM 残差插值 (恒等安全)
      - 'gated' : 乘法门控残差 (恒等安全)
      - 'concat': 拼接后 MLP
    """
    def __init__(self, z_dim: int, c_dim: int, out_dim: int, mode: str = "film"):
        super().__init__()
        self.mode = mode.lower()
        assert self.mode in ("film", "gated", "concat")

        if self.mode == "film":
            self.film = FiLM1d(z_dim, c_dim, gamma_scale=0.1, beta_scale=0.1)
            self.alpha = nn.Parameter(torch.tensor(0.0))  # 残差系数，0 起步=恒等
            self.post = nn.Sequential(nn.LayerNorm(z_dim), nn.ReLU(inplace=True),
                                      nn.Linear(z_dim, out_dim), nn.LayerNorm(out_dim), nn.ReLU(inplace=True))
        elif self.mode == "gated":
            self.gate = nn.Linear(c_dim, z_dim)
            nn.init.zeros_(self.gate.weight); nn.init.zeros_(self.gate.bias)  # 恒等起点
            self.alpha = nn.Parameter(torch.tensor(0.0))
            self.post = nn.Sequential(nn.LayerNorm(z_dim), nn.ReLU(inplace=True),
                                      nn.Linear(z_dim, out_dim), nn.LayerNorm(out_dim), nn.ReLU(inplace=True))
        else:  # concat
            self.post = nn.Sequential(
                nn.Linear(z_dim + c_dim, 512), nn.LayerNorm(512), nn.ReLU(inplace=True),
                nn.Linear(512, out_dim), nn.LayerNorm(out_dim), nn.ReLU(inplace=True)
            )

    def forward(self, z: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
        if self.mode == "film":
            z_tilde = self.film(z, c)
            z = z + self.alpha * (z_tilde - z)
            return self.post(z)
        elif self.mode == "gated":
            g = torch.tanh(self.gate(c)) * 0.1         # 小尺度门控，初始≈0
            z_tilde = z * (1 + g)
            z = z + self.alpha * (z_tilde - z)
            return self.post(z)
        else:  # concat
            return self.post(torch.cat([z, c], dim=1))

# ---------------- Custom CNN (pure late fusion) ----------------
class CustomCNN(BaseFeaturesExtractor):
    """
    观测仍为: [ costmaps(T * H*W) , gate(1) , goal(2) , history_action(action_space) ]
    仅在 head 做后连接融合；主干不注入条件。
    """
    def __init__(
        self,
        observation_space: gym.spaces.Box,
        features_dim: int = 256,
        costmap_width: int = 100,
        num_costmap_frames: int = 2,
        action_space: int = 4,
        apply_gate_on_dynamic: bool = True,
        cond_dim: int = 64,
        goal_include_theta: bool = False,
        late_mode: str = "film"   # 'film' | 'gated' | 'concat'
    ):
        super().__init__(observation_space, features_dim)

        self.H = int(costmap_width)
        self.W = int(costmap_width)
        self.per_map_dim = self.H * self.W
        self.T = int(num_costmap_frames)
        self.gate_dim = 1
        self.goal_dim = 2
        self.action_space = int(action_space)
        self.apply_gate_on_dynamic = apply_gate_on_dynamic and (self.T >= 2)

        expected = self.T * self.per_map_dim + self.gate_dim + self.goal_dim + self.action_space
        assert observation_space.shape == (expected,), \
            f"obs 维度不匹配: 期望 {expected}, 实际 {observation_space.shape[0]}"

        # -------- trunk: only costmaps --------
        self.inplanes = 64
        self.conv1_time = nn.Conv3d(1, self.inplanes, kernel_size=(3,3,3), stride=1, padding=1, bias=True)
        self.ln1   = ChannelLayerNorm(self.inplanes, affine=True)  # 仅特征归一
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)

        block  = Bottleneck
        layers = [2, 1, 1]
        self.dilation = 1
        self.groups   = 1
        self.base_width = 64

        self.layer1 = self._make_layer(block, 64,  layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)

        self.conv2_2 = nn.Sequential(
            nn.Conv2d(256, 128, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1, bias=True),
            ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(256)
        )
        self.downsample2 = nn.Sequential(
            nn.Conv2d(128, 256, kernel_size=1, stride=2, padding=0, bias=True),
            ChannelLayerNorm(256)
        )
        self.relu2 = nn.ReLU(inplace=True)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(512, 256, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(256), nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1, bias=True),
            ChannelLayerNorm(256), nn.ReLU(inplace=True),
            nn.Conv2d(256, 512, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(512)
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(64, 512, kernel_size=1, stride=4, padding=0, bias=True),
            ChannelLayerNorm(512)
        )
        self.relu3 = nn.ReLU(inplace=True)

        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))

        # -------- condition path (gate + goal + history_action) --------
        cond_in = self.gate_dim + self.goal_dim + self.action_space
        self.cond_mlp = nn.Sequential(nn.Linear(cond_in, cond_dim), nn.ReLU(inplace=True))

        # -------- late fusion head --------
        self.fusion_head = LateFusionHead(z_dim=512, c_dim=cond_dim, out_dim=features_dim, mode=late_mode)

        # init
        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.Conv3d)):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.Linear):
                # 已在 LateFusionHead/FiLM1d 内对敏感层做了零初始化
                if m.weight is not getattr(getattr(self, 'fusion_head', None), 'film', None):
                    nn.init.xavier_normal_(m.weight)

    def _make_layer(self, block, planes, blocks, stride=1, dilate=False):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                ChannelLayerNorm(planes * block.expansion)
            )
        layers = []
        layers.append(block(
            self.inplanes, planes, stride=stride, downsample=downsample,
            groups=self.groups, base_width=self.base_width, dilation=self.dilation, norm_layer=None
        ))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(
                self.inplanes, planes,
                groups=self.groups, base_width=self.base_width, dilation=self.dilation, norm_layer=None
            ))
        return nn.Sequential(*layers)

    def _forward_impl(self, costmaps_2d_list, gate, goal, hist_action):
        N = gate.shape[0]

        # 仍可选择在最后一帧（动态帧）上应用 gate 掩蔽
        if self.apply_gate_on_dynamic:
            costmaps_2d_list[-1] = costmaps_2d_list[-1] * gate.view(N, 1, 1, 1)

        # 3D -> 2D trunk
        cost_map_in = torch.stack(costmaps_2d_list, dim=2)   # [N, 1, T, H, W]
        x = self.conv1_time(cost_map_in)                     # [N, C, T, H, W]
        x = x.sum(dim=2)                                     # 时间聚合: sum
        x = self.ln1(x); x = self.relu(x); x = self.maxpool(x)

        identity3 = self.downsample3(x)
        x = self.layer1(x)
        identity2 = self.downsample2(x)
        x = self.layer2(x)
        x = self.conv2_2(x); x += identity2; x = self.relu2(x)
        x = self.layer3(x)
        x = self.conv3_2(x); x += identity3; x = self.relu3(x)

        x = self.avgpool(x)
        z = torch.flatten(x, 1)                              # [N, 512]

        # condition embedding
        cond = torch.cat([gate, goal, hist_action], dim=1)   # [N, 1+2+action_space]
        c = self.cond_mlp(cond)                              # [N, cond_dim]

        # late fusion
        out = self.fusion_head(z, c)                         # [N, features_dim]
        return out

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        N = observations.shape[0]
        offset = 0
        costmaps_2d_list = []
        for _ in range(self.T):
            cm = observations[:, offset: offset + self.per_map_dim]
            offset += self.per_map_dim
            costmaps_2d_list.append(cm.view(N, 1, self.H, self.W))

        gate = observations[:, offset: offset + self.gate_dim]; offset += self.gate_dim
        goal = observations[:, offset: offset + self.goal_dim]; offset += self.goal_dim
        hist = observations[:, offset: offset + self.action_space]; offset += self.action_space
        return self._forward_impl(costmaps_2d_list, gate, goal, hist)

# ============================== 简单自测（替换原 __main__） ==============================
if __name__ == "__main__":
    set_seed(SEED1)

    costmap_width = 50
    T = 2
    action_space = 8
    obs_dim = T * (costmap_width**2) + 1 + 2 + action_space

    observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
    features_dim = 256

    model = CustomCNN(
        observation_space,
        features_dim=features_dim,
        costmap_width=costmap_width,
        num_costmap_frames=T,
        action_space=action_space,
        apply_gate_on_dynamic=True,  # IL 时动态通道本就为 0，开/关均可
        cond_dim=64,
        late_mode="film"             # 推荐：'film'；可改为 'gated' 或 'concat'
    )

    batch_size = 4
    dummy_input = torch.randn(batch_size, obs_dim)
    with torch.no_grad():
        output = model(dummy_input)
    print("Input shape :", dummy_input.shape)
    print("Output shape:", output.shape)     # (4, 256)
