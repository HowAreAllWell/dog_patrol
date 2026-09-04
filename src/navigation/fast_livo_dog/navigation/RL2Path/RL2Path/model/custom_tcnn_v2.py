import math
import os
import random
import numpy as np

import torch
import torch.nn as nn

import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# ---------------- Utils ----------------
SEED1 = 3047
def set_seed(seed: int):
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)

# ---------------- LayerNorm over channels for NCHW ----------------
class ChannelLayerNorm(nn.Module):
    """LayerNorm over channels only for input (N, C, H, W)."""
    def __init__(self, num_channels: int, eps: float = 1e-5, affine: bool = True):
        super().__init__()
        self.ln = nn.LayerNorm(normalized_shape=num_channels, eps=eps,
                               elementwise_affine=affine)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.permute(0, 2, 3, 1)
        x = self.ln(x)
        x = x.permute(0, 3, 1, 2)
        return x

# ---------------- ResNet bits ----------------
def conv3x3(in_planes, out_planes, stride: int = 1, groups: int = 1, dilation: int = 1):
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=dilation, groups=groups, bias=True, dilation=dilation)

def conv1x1(in_planes, out_planes, stride: int = 1):
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=True)

class Bottleneck(nn.Module):
    expansion = 2
    def __init__(self, inplanes, planes, stride: int = 1, downsample=None,
                 groups: int = 1, base_width: int = 64, dilation: int = 1):
        super().__init__()
        width = int(planes * (base_width / 64.)) * groups
        self.conv1 = conv1x1(inplanes, width)
        self.ln1   = ChannelLayerNorm(width)
        self.conv2 = conv3x3(width, width, stride=stride, groups=groups, dilation=dilation)
        self.ln2   = ChannelLayerNorm(width)
        self.conv3 = conv1x1(width, planes * self.expansion)
        self.ln3   = ChannelLayerNorm(planes * self.expansion)
        self.relu  = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = x
        out = self.conv1(x); out = self.ln1(out); out = self.relu(out)
        out = self.conv2(out); out = self.ln2(out); out = self.relu(out)
        out = self.conv3(out); out = self.ln3(out)
        if self.downsample is not None:
            identity = self.downsample(x)
        out += identity
        return self.relu(out)

# ---------------- Fusion head (single big MLP) ----------------
class FusionMLP(nn.Module):
    """Concat trunk vector z & raw cond vector (goal + history) → 2-layer MLP."""
    def __init__(self, z_dim: int, c_dim: int, out_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(z_dim + c_dim, 512),
            nn.LayerNorm(512),
            nn.GELU(),
            nn.Linear(512, out_dim),
            nn.LayerNorm(out_dim),
            nn.GELU()
        )

    def forward(self, z: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
        return self.net(torch.cat([z, c], dim=1))

# ---------------- Custom CNN (late fusion by concat) ----------------
class CustomCNN(BaseFeaturesExtractor):
    """
    Observation vector:
        [ costmaps(T * H*W) , goal(dataset_goal_dim) , history_action(action_space) ]
    - 数据集的 goal 维度固定为 dataset_goal_dim (默认 2: [rho, theta])。
    - 当 goal_include_theta=False 时，网络仍然从观测中读取并跳过 dataset_goal_dim 个位置，
      但只取其中的 rho（前 1 维）进入融合头，以保持观测维度对齐且不使用 theta。
    Trunk handles costmaps; (goal + history) only enter at final fusion MLP.
    """
    def __init__(self,
                 observation_space: gym.spaces.Box,
                 features_dim: int = 256,
                 costmap_width: int = 100,
                 num_costmap_frames: int = 2,
                 action_space: int = 4,
                 goal_include_theta: bool = True,
                 dataset_goal_dim: int = 2):
        super().__init__(observation_space, features_dim)

        # ------------ dimensions ------------
        self.H = costmap_width
        self.W = costmap_width
        self.per_map_dim = self.H * self.W
        self.T = num_costmap_frames

        print(f"[INFO] customcnn goal_include_theta: {goal_include_theta}")
        
        # 数据集中的 goal 维度（原始占位数）
        assert dataset_goal_dim >= 1, "dataset_goal_dim must be >= 1"
        if goal_include_theta:
            assert dataset_goal_dim >= 2, \
                "When goal_include_theta=True, dataset_goal_dim must be at least 2 (e.g., [rho, theta])."
        self.dataset_goal_dim = int(dataset_goal_dim)

        # 实际用于网络融合的 goal 维度（1: 仅 rho；2: rho+theta）
        self.goal_include_theta = bool(goal_include_theta)
        self.goal_dim = 2 if self.goal_include_theta else 1

        self.action_space = action_space

        # 观测维度检查：以数据集的占位维度为准
        expected = self.T * self.per_map_dim + self.dataset_goal_dim + self.action_space
        assert observation_space.shape == (expected,), \
            (f"Observation dim mismatch: expected {expected} "
             f"(T={self.T}, H=W={self.H}, dataset_goal_dim={self.dataset_goal_dim}, action_space={self.action_space}), "
             f"but got {tuple(observation_space.shape)}")

        # ------------ trunk (3D conv + small ResNet) ------------
        self.inplanes = 64
        self.conv1_time = nn.Conv3d(1, self.inplanes,
                                    kernel_size=(3,3,3), stride=1, padding=1, bias=True)
        self.ln1   = ChannelLayerNorm(self.inplanes)
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)

        block, layers_cfg = Bottleneck, [2, 1, 1]
        self.dilation, self.groups, self.base_width = 1, 1, 64

        self.layer1 = self._make_layer(block, 64,  layers_cfg[0])
        self.layer2 = self._make_layer(block, 128, layers_cfg[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers_cfg[2], stride=2)

        self.conv2_2 = nn.Sequential(
            nn.Conv2d(256, 128, kernel_size=1, bias=True),
            ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, kernel_size=3, padding=1, bias=True),
            ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, kernel_size=1, bias=True),
            ChannelLayerNorm(256)
        )
        self.downsample2 = nn.Sequential(
            nn.Conv2d(128, 256, kernel_size=1, stride=2, bias=True),
            ChannelLayerNorm(256)
        )
        self.relu2 = nn.ReLU(inplace=True)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(512, 256, kernel_size=1, bias=True),
            ChannelLayerNorm(256), nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1, bias=True),
            ChannelLayerNorm(256), nn.ReLU(inplace=True),
            nn.Conv2d(256, 512, kernel_size=1, bias=True),
            ChannelLayerNorm(512)
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(64, 512, kernel_size=1, stride=4, bias=True),
            ChannelLayerNorm(512)
        )
        self.relu3 = nn.ReLU(inplace=True)

        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))

        # ------------ fusion head ------------
        cond_input_dim = self.goal_dim + self.action_space
        self.fusion_head = FusionMLP(z_dim=512, c_dim=cond_input_dim, out_dim=features_dim)

        # ------------ init ------------
        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.Conv3d)):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
            elif isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    # -----------------------------------------------------------------
    def _make_layer(self, block, planes, blocks, stride: int = 1):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                ChannelLayerNorm(planes * block.expansion)
            )

        layers = [block(self.inplanes, planes, stride=stride, downsample=downsample,
                        groups=self.groups, base_width=self.base_width, dilation=self.dilation)]
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes,
                                groups=self.groups, base_width=self.base_width, dilation=self.dilation))
        return nn.Sequential(*layers)

    # -----------------------------------------------------------------
    def _forward_impl(self, costmaps_2d_list, goal, hist_action):
        N = goal.size(0)

        # 3D-->2D trunk (sum over time)
        cost_map_in = torch.stack(costmaps_2d_list, dim=2)  # (N,1,T,H,W)
        x = self.conv1_time(cost_map_in)                    # (N,64,T,H,W)
        x = x.sum(dim=2)                                    # (N,64,H,W)
        x = self.ln1(x); x = self.relu(x); x = self.maxpool(x)

        identity3 = self.downsample3(x)
        x = self.layer1(x)
        identity2 = self.downsample2(x)
        x = self.layer2(x)
        x = self.conv2_2(x); x = x + identity2; x = self.relu2(x)
        x = self.layer3(x)
        x = self.conv3_2(x); x = x + identity3; x = self.relu3(x)

        x = self.avgpool(x)
        z = torch.flatten(x, 1)                             # (N,512)

        cond_vec = torch.cat([goal, hist_action], dim=1)    # (N,goal_used + hist)
        return self.fusion_head(z, cond_vec)                # (N,features_dim)

    # -----------------------------------------------------------------
    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        N, offset = observations.size(0), 0
        costmaps_2d_list = []
        for _ in range(self.T):
            cm = observations[:, offset: offset + self.per_map_dim]
            offset += self.per_map_dim
            costmaps_2d_list.append(cm.view(N, 1, self.H, self.W))

        # —— 关键修改：始终跳过 “数据集的 goal 占位维度” —— #
        goal_raw = observations[:, offset: offset + self.dataset_goal_dim]
        offset += self.dataset_goal_dim  # 不再用 self.goal_dim 来推进 offset

        # 根据开关决定实际送入融合头的 goal 维度
        if self.goal_include_theta:
            goal = goal_raw  # 形如 [rho, theta]
        else:
            # 仅使用 rho（默认取第 0 维）；如需改成用第 1 维，请改为 goal_raw[:, 1:2]
            goal = goal_raw[:, :1]  # 保持二维形状 (N,1)

        hist = observations[:, offset: offset + self.action_space]
        return self._forward_impl(costmaps_2d_list, goal, hist)

    # -----------------------------------------------------------------
    @staticmethod
    def expected_obs_dim(costmap_width: int, num_costmap_frames: int,
                         action_space: int, dataset_goal_dim: int = 2) -> int:
        """仅用于根据‘数据集真实维度’计算观测长度。"""
        per_map = costmap_width * costmap_width
        return num_costmap_frames * per_map + dataset_goal_dim + action_space

# ============================== quick self-test ==============================
if __name__ == "__main__":
    set_seed(SEED1)

    costmap_width = 55
    T = 2
    action_space = 12
    features_dim = 256
    dataset_goal_dim = 2  # 数据集里固定为 2（[rho, theta]）

    # --------- Case A: 使用 theta（goal_dim 实际=2），obs 维度不变 ---------
    obs_dim = CustomCNN.expected_obs_dim(
        costmap_width=costmap_width,
        num_costmap_frames=T,
        action_space=action_space,
        dataset_goal_dim=dataset_goal_dim
    )
    observation_space_A = gym.spaces.Box(low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
    model_A = CustomCNN(
        observation_space_A,
        features_dim=features_dim,
        costmap_width=costmap_width,
        num_costmap_frames=T,
        action_space=action_space,
        goal_include_theta=True,          # 使用 theta
        dataset_goal_dim=dataset_goal_dim # 观测占位仍为 2
    )
    batch_size = 4
    dummy_input_A = torch.randn(batch_size, obs_dim)
    with torch.no_grad():
        output_A = model_A(dummy_input_A)
    print("[A] include theta | Input:", dummy_input_A.shape, " Output:", output_A.shape)

    # --------- Case B: 不使用 theta（goal_dim 实际=1），obs 维度仍不变 ---------
    observation_space_B = gym.spaces.Box(low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
    model_B = CustomCNN(
        observation_space_B,
        features_dim=features_dim,
        costmap_width=costmap_width,
        num_costmap_frames=T,
        action_space=action_space,
        goal_include_theta=False,         # 忽略 theta
        dataset_goal_dim=dataset_goal_dim # 观测占位仍为 2
    )
    dummy_input_B = torch.randn(batch_size, obs_dim)
    with torch.no_grad():
        output_B = model_B(dummy_input_B)
    print("[B] exclude theta | Input:", dummy_input_B.shape, " Output:", output_B.shape)