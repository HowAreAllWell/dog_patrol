import torch
import torch.nn as nn
import torch.nn.functional as F
import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import random, os, numpy as np


# --------------------------------------------------------------------------
#  Channel-wise LayerNorm（与原版相同）
# --------------------------------------------------------------------------
class ChannelLayerNorm(nn.Module):
    def __init__(self, num_channels, eps=1e-5, affine=True):
        super().__init__()
        self.ln = nn.LayerNorm(num_channels, eps=eps, elementwise_affine=affine)

    def forward(self, x):                         # x: (N,C,H,W)
        return self.ln(x.permute(0, 2, 3, 1)).permute(0, 3, 1, 2)


# --------------------------------------------------------------------------
#  3×3 / 1×1 卷积快捷函数
# --------------------------------------------------------------------------
def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    return nn.Conv2d(
        in_planes, out_planes,
        kernel_size=3,
        stride=stride,
        padding=dilation,     # 只传一次
        dilation=dilation,
        groups=groups,
        bias=True
    )

def conv1x1(in_planes, out_planes, stride=1):
    return nn.Conv2d(in_planes, out_planes, 1, stride, padding=0, bias=True)


# --------------------------------------------------------------------------
#  Bottleneck 残差块（与原版相同，expansion=2）
# --------------------------------------------------------------------------
class Bottleneck(nn.Module):
    expansion = 2

    def __init__(self, inplanes, planes, stride=1, downsample=None,
                 groups=1, base_width=32, dilation=1):
        super().__init__()
        width = int(planes * (base_width / 64.)) * groups   # ← base_width 改成 32 会自动减半

        self.conv1 = conv1x1(inplanes, width)
        self.ln1   = ChannelLayerNorm(width)
        self.conv2 = conv3x3(width,   width, stride=stride, groups=groups, dilation=dilation)
        self.ln2   = ChannelLayerNorm(width)
        self.conv3 = conv1x1(width, planes * self.expansion)
        self.ln3   = ChannelLayerNorm(planes * self.expansion)
        self.relu  = nn.ReLU(inplace=True)

        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x

        out = self.relu(self.ln1(self.conv1(x)))
        out = self.relu(self.ln2(self.conv2(out)))
        out = self.ln3(self.conv3(out))

        if self.downsample is not None:
            identity = self.downsample(x)

        return self.relu(out + identity)


# --------------------------------------------------------------------------
#  MiniCustomCNN：参数精简版
# --------------------------------------------------------------------------
class MiniCustomCNN(BaseFeaturesExtractor):
    """
    - 仍以 3 帧 50×50 cost-map + 2 维 goal 作为输入
    - features_dim 默认 256，可自行调小
    """
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super().__init__(observation_space, features_dim)

        block         = Bottleneck
        layers        = [1, 1, 1]   # 每个 stage 只保留 1 个 bottleneck
        self.inplanes = 32          # stem 输出通道减半
        self.groups   = 1
        self.base_width = 32        # bottleneck 中间宽度降半
        self.dilation = 1

        # --- 3D 时间卷积（仍保留） ---
        self.conv1_time = nn.Conv3d(
            1, self.inplanes, kernel_size=(3,3,3), padding=1, bias=True
        )
        self.ln1   = ChannelLayerNorm(self.inplanes)
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(3, stride=1, padding=1)

        # --- ResNet-like 主干 ---
        self.layer1 = self._make_layer(block, 32,  layers[0])          # 输出  64
        self.layer2 = self._make_layer(block, 64,  layers[1], stride=2)# 输出 128
        self.layer3 = self._make_layer(block, 128, layers[2], stride=2)# 输出 256

        # --- 两段额外的 conv-BN-ReLU-conv-skip，同样按比例缩 ---
        self.conv2_2 = nn.Sequential(
            nn.Conv2d(128, 64, 1, bias=True),  ChannelLayerNorm(64),  nn.ReLU(inplace=True),
            nn.Conv2d(64,  64, 3, padding=1, bias=True), ChannelLayerNorm(64), nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 1, bias=True),  ChannelLayerNorm(128)
        )
        self.downsample2 = nn.Sequential(
            nn.Conv2d(64, 128, 1, stride=2, bias=True), ChannelLayerNorm(128)
        )
        self.relu2 = nn.ReLU(inplace=True)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(256, 128, 1, bias=True), ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128,128, 3, padding=1, bias=True), ChannelLayerNorm(128), nn.ReLU(inplace=True),
            nn.Conv2d(128,256, 1, bias=True), ChannelLayerNorm(256)
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(32, 256, 1, stride=4, bias=True), ChannelLayerNorm(256)
        )
        self.relu3 = nn.ReLU(inplace=True)

        # --- 全局汇聚 + FC ---
        self.avgpool   = nn.AdaptiveAvgPool2d((1,1))
        self.linear_fc = nn.Sequential(
            nn.Linear(256 + 2, features_dim),
            nn.LayerNorm(features_dim),
            nn.ReLU(inplace=True)
        )

        # 权重初始化（与原版一致）
        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.Conv3d)):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)

    # ----------------------------------------------------------------------
    #  工具函数
    # ----------------------------------------------------------------------
    def _make_layer(self, block, planes, blocks, stride=1, dilate=False):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                ChannelLayerNorm(planes * block.expansion)
            )

        layers = [block(self.inplanes, planes, stride, downsample,
                        self.groups, self.base_width, self.dilation)]
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes,
                                groups=self.groups, base_width=self.base_width,
                                dilation=self.dilation))
        return nn.Sequential(*layers)

    # ----------------------------------------------------------------------
    #  前向
    # ----------------------------------------------------------------------
    def _forward_impl(self, cost_map, goal):
        x = self.conv1_time(cost_map.view(-1,1,3,50,50))   # (N,32,3,50,50)
        x = self.ln1(x.mean(dim=2))                        # 时间维度平均→(N,32,50,50)
        x = self.relu(self.maxpool(x))

        identity3 = self.downsample3(x)

        x = self.layer1(x)

        identity2 = self.downsample2(x)
        x = self.layer2(x)

        x = self.relu2(self.conv2_2(x) + identity2)

        x = self.layer3(x)
        x = self.relu3(self.conv3_2(x) + identity3)

        fusion_out = torch.flatten(self.avgpool(x), 1)     # (N,256)

        out = self.linear_fc(torch.cat([fusion_out, goal.view(-1,2)], dim=1))
        return out

    def forward(self, observations: torch.Tensor):
        return self._forward_impl(observations[:,:7500], observations[:,7500:])


# ----------------------------------------------------------------------------
#  简单 sanity-check
# ----------------------------------------------------------------------------
if __name__ == "__main__":
    obs_shape = (7502,)
    space = gym.spaces.Box(low=-1., high=1., shape=obs_shape, dtype=np.float32)

    net = MiniCustomCNN(space, features_dim=128)  # 也可把 features_dim 调得更小
    x = torch.randn(4, 7502)
    y = net(x)
    print("Output shape:", y.shape)               # → (4,128)
