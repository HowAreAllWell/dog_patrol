import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import numpy.matlib
import os
import random

# 环境相关
import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

# -----------------------------------------------------------------------------
# 全局随机数种子
# -----------------------------------------------------------------------------
SEED1 = 3407

def set_seed(seed):
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)

# -----------------------------------------------------------------------------
# 定义一个用于替代 BatchNorm2d 的简单 LayerNorm：只对通道维度做归一化
# -----------------------------------------------------------------------------
class ChannelLayerNorm(nn.Module):
    """
    对输入 (N, C, H, W) 做 LayerNorm，但只在通道维度进行归一化（每个通道在 H*W 上计算均值方差）。
    如果想对 (C, H, W) 全部做归一化，需要在 forward 中先 permute，然后传入合适的 normalized_shape。
    """
    def __init__(self, num_channels, eps=1e-5, affine=True):
        super().__init__()
        self.ln = nn.LayerNorm(normalized_shape=num_channels, 
                               eps=eps, 
                               elementwise_affine=affine)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x.shape = (N, C, H, W)
        N, C, H, W = x.shape
        # 转成 (N, H, W, C)，让 C 成为最后一维
        x = x.permute(0, 2, 3, 1)
        # 在最后一维上做 LayerNorm
        x = self.ln(x)
        # 转回 (N, C, H, W)
        x = x.permute(0, 3, 1, 2)
        return x

# -----------------------------------------------------------------------------
# ResNet相关函数与模块 (conv3x3, conv1x1, Bottleneck)
# -----------------------------------------------------------------------------
def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    """3x3 convolution with padding"""
    return nn.Conv2d(
        in_planes, 
        out_planes, 
        kernel_size=3, 
        stride=stride,
        padding=dilation, 
        groups=groups, 
        bias=True,   # 去掉BN后可将bias设为True
        dilation=dilation
    )

def conv1x1(in_planes, out_planes, stride=1):
    """1x1 convolution"""
    return nn.Conv2d(
        in_planes, 
        out_planes, 
        kernel_size=1, 
        stride=stride, 
        bias=True   # 去掉BN后可将bias设为True
    )

class Bottleneck(nn.Module):
    # 原论文中的扩张系数，这里是2
    expansion = 2

    def __init__(self, inplanes, planes, stride=1, downsample=None, 
                 groups=1, base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()
        # 这里的 norm_layer 不再使用 BN，而是使用 ChannelLayerNorm
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

# -----------------------------------------------------------------------------
# 自定义CNN (替换了所有 BN 为 LN)
# -----------------------------------------------------------------------------
class CustomCNN(BaseFeaturesExtractor):
    """
    将原先的BN全部去除，改用 LayerNorm（这里以自定义的 ChannelLayerNorm 为例）。
    """
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super(CustomCNN, self).__init__(observation_space, features_dim)

        # ResNet相关配置
        block  = Bottleneck
        layers = [2, 1, 1]
        self.inplanes = 64
        self.dilation = 1
        self.groups   = 1
        self.base_width = 64

        # 第一层卷积 + LayerNorm + ReLU
        self.conv1 = nn.Conv2d(1, self.inplanes, kernel_size=3, stride=1, 
                               padding=1, bias=True)
        self.ln1   = ChannelLayerNorm(self.inplanes)
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)

        # 构造后续 layers
        self.layer1 = self._make_layer(block, 64,  layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)

        # conv2_2 分支
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

        # conv3_2 分支
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
            nn.Linear(256 * block.expansion + 2, features_dim),
            # LayerNorm 或直接去掉
            nn.LayerNorm(features_dim),
            nn.ReLU()
        )

        # 权重初始化
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
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
            norm_layer=None   # 不再传 BN
        ))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(
                self.inplanes, planes, 
                groups=self.groups, base_width=self.base_width, dilation=self.dilation,
                norm_layer=None
            ))

        return nn.Sequential(*layers)

    def _forward_impl(self, cost_map, goal):
        # cost_map: [batch_size, 2500] -> reshape -> (N, 1, 50, 50)
        cost_map_in = cost_map.view(-1, 1, 50, 50)
        x = self.conv1(cost_map_in)
        x = self.ln1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        identity3 = self.downsample3(x)
        x = self.layer1(x)

        identity2 = self.downsample2(x)
        x = self.layer2(x)

        x = self.conv2_2(x)
        x += identity2
        x = self.relu2(x)

        x = self.layer3(x)
        x = self.conv3_2(x)
        x += identity3
        x = self.relu3(x)

        x = self.avgpool(x)
        fusion_out = torch.flatten(x, 1)

        # goal: [batch_size, 2]
        goal_in = goal.view(-1, 2)
        goal_out = torch.flatten(goal_in, 1)

        fc_in = torch.cat((fusion_out, goal_out), dim=1)
        out = self.linear_fc(fc_in)
        return out

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        cost_map = observations[:, :2500]
        goal = observations[:, 2500:]
        return self._forward_impl(cost_map, goal)


# -----------------------------------------------------------------------------
# 测试示例
# -----------------------------------------------------------------------------
def test_custom_cnn():
    # 定义观察空间：2500 (cost map) + 2 (local goal) = 2502
    observation_space = gym.spaces.Box(low=0, high=1, shape=(2502,), dtype=np.float32)

    # 实例化 CustomCNN
    features_dim = 256
    model = CustomCNN(observation_space, features_dim=features_dim)

    # 将模型设置为评估模式
    model.eval()

    # 创建一个虚拟的输入张量，假设批量大小为4
    batch_size = 4
    # 随机生成成本图和目标
    cost_map = np.random.rand(batch_size, 2500).astype(np.float32)
    goal = np.random.rand(batch_size, 2).astype(np.float32)
    # 合并成本图和目标
    observations = np.concatenate((cost_map, goal), axis=1)
    observations_tensor = torch.from_numpy(observations)

    # 前向传播
    with torch.no_grad():
        output = model(observations_tensor)

    print("Input shape:", observations_tensor.shape)  # 应为 (4, 2502)
    print("Output shape:", output.shape)              # 应为 (4, features_dim)
    print("Output content:", output)


if __name__ == "__main__":
    set_seed(SEED1)
    test_custom_cnn()
