import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import os
import random

import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# --------------------------------------------------------------------------
# 全局随机数种子
# --------------------------------------------------------------------------
SEED1 = 3407
VOXEL_D  = VOXEL_H = VOXEL_W = 50
VOXEL_FLAT = VOXEL_D * VOXEL_H * VOXEL_W        # 125 000
OBS_DIM    = VOXEL_FLAT + 3                     # 125 003

def set_seed(seed):
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)

# --------------------------------------------------------------------------
# 定义一个用于替代 BatchNorm2d 的简单 LayerNorm：只对通道维度做归一化
# --------------------------------------------------------------------------
class ChannelLayerNorm(nn.Module):
    """
    对输入 (N, C, H, W) 做 LayerNorm，但只在通道维度进行归一化。
    """
    def __init__(self, num_channels, eps=1e-5, affine=True):
        super().__init__()
        self.ln = nn.LayerNorm(normalized_shape=num_channels, 
                               eps=eps, 
                               elementwise_affine=affine)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x.shape = (N, C, H, W)
        N, C, H, W = x.shape
        # 转成 (N, H, W, C)
        x = x.permute(0, 2, 3, 1)
        # 在最后一维上做 LayerNorm
        x = self.ln(x)
        # 转回 (N, C, H, W)
        x = x.permute(0, 3, 1, 2)
        return x

# --------------------------------------------------------------------------
# ResNet相关函数与模块
# --------------------------------------------------------------------------
def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    """3x3 convolution with padding"""
    return nn.Conv2d(
        in_planes, 
        out_planes, 
        kernel_size=3, 
        stride=stride,
        padding=dilation, 
        groups=groups, 
        bias=True,
        dilation=dilation
    )

def conv1x1(in_planes, out_planes, stride=1):
    """1x1 convolution"""
    return nn.Conv2d(in_planes, out_planes, 
                     kernel_size=1, stride=stride, bias=True)

class Bottleneck(nn.Module):
    expansion = 2

    def __init__(self, inplanes, planes, stride=1, downsample=None, 
                 groups=1, base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()
        width = int(planes * (base_width / 64.)) * groups

        self.conv1 = conv1x1(inplanes, width, stride=1)
        self.ln1   = ChannelLayerNorm(width)
        self.conv2 = conv3x3(width, width, stride=stride, 
                             groups=groups, dilation=dilation)
        self.ln2   = ChannelLayerNorm(width)
        self.conv3 = conv1x1(width, planes * self.expansion, stride=1)
        self.ln3   = ChannelLayerNorm(planes * self.expansion)

        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x

        out = self.conv1(x)
        out = self.ln1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.ln2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.ln3(out)

        if self.downsample is not None:
            identity = self.downsample(x)

        out += identity
        out = self.relu(out)
        return out

# --------------------------------------------------------------------------
# 自定义CNN (替换BN为LN, 并在最前面做时间卷积)
# --------------------------------------------------------------------------
class CustomCNN(BaseFeaturesExtractor):
    """
    输入维度变为 7502:
    - 前7500: 3帧costmap，每帧2500 = 50×50
    - 后2:    goal坐标
    """
    def __init__(self,
                 observation_space: gym.spaces.Box,
                 features_dim: int = 256):
        super().__init__(observation_space, features_dim)

        # ResNet 配置保持不变 ---------------------------
        block, layers = Bottleneck, [2,1,1]
        self.inplanes, self.dilation = 64, 1
        self.groups, self.base_width = 1, 64

        # --------- ❶ 3D 卷积适配新 Voxel 输入 ---------
        self.conv1_3d = nn.Conv3d(1, self.inplanes,
                                  kernel_size=3, stride=1, padding=1, bias=True)
        self.ln1   = ChannelLayerNorm(self.inplanes)
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)

        # 接下来的网络结构仍然是2D卷积
        self.layer1 = self._make_layer(block, 64,  layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)

        # 下面和原先一致
        self.conv2_2 = nn.Sequential(
            nn.Conv2d(256, 128, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1, bias=True),
            ChannelLayerNorm(128),
            nn.ReLU(inplace=True),
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
            ChannelLayerNorm(256),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1, bias=True),
            ChannelLayerNorm(256),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 512, kernel_size=1, stride=1, padding=0, bias=True),
            ChannelLayerNorm(512)
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(64, 512, kernel_size=1, stride=4, padding=0, bias=True),
            ChannelLayerNorm(512)
        )
        self.relu3 = nn.ReLU(inplace=True)

        # 池化 + 全连接
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.linear_fc = nn.Sequential(
            nn.Linear(256 * block.expansion + 3, features_dim),
            nn.LayerNorm(features_dim),
            nn.ReLU()
        )

        # 权重初始化
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.Conv3d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.Linear):
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
            groups=self.groups, base_width=self.base_width, dilation=self.dilation,
            norm_layer=None
        ))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(
                self.inplanes, planes, 
                groups=self.groups, base_width=self.base_width, dilation=self.dilation,
                norm_layer=None
            ))
        return nn.Sequential(*layers)

    def _forward_impl(self, voxel_flat, goal):
        """
        voxel_flat: [B, 125000]  -> reshape -> (B, 1, 50, 50, 50)
        goal      : [B, 2]
        """
        x = voxel_flat.view(-1, 1, VOXEL_D, VOXEL_H, VOXEL_W)  # 3D input

        x = self.conv1_3d(x)          # (B,64,50,50,50)
        x = x.mean(dim=2)             # 深度维平均 → (B,64,50,50)

        x = self.ln1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        # 原 2D ResNet 流水线 -------------------------
        identity3 = self.downsample3(x)
        x = self.layer1(x)

        identity2 = self.downsample2(x)
        x = self.layer2(x)

        x = self.conv2_2(x); x += identity2; x = self.relu2(x)

        x = self.layer3(x)
        x = self.conv3_2(x); x += identity3; x = self.relu3(x)

        x = self.avgpool(x)
        cnn_out = torch.flatten(x, 1)        # (B, 256*expansion)

        out = self.linear_fc(torch.cat([cnn_out, goal], dim=1))
        return out

    # ========= 外部 forward =========
    def forward(self, obs):
        voxel_flat = obs[:, :VOXEL_FLAT]  # 125000
        goal       = obs[:, VOXEL_FLAT:]  # 3
        return self._forward_impl(voxel_flat, goal)

# ---------------------------------------------
# 用法示例
if __name__ == "__main__":
    import numpy as np
    obs_space = gym.spaces.Box(low=-1., high=1., shape=(OBS_DIM,), dtype=np.float32)
    net = CustomCNN(obs_space, features_dim=256)

    dummy = torch.randn(4, OBS_DIM)   # batch_size = 4
    with torch.no_grad():
        y = net(dummy)
    print(y.shape)    # -> torch.Size([4, 256])