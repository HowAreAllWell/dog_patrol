import torch
import torch.nn as nn
import gymnasium as gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class CustomCNN(BaseFeaturesExtractor):
    """
    Custom ResNet-based feature extractor for processing a 50x50 costmap and a 2D goal point.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        # Assuming observation_space is a Box with shape (50*50 + 2,)
        super(CustomCNN, self).__init__(observation_space, features_dim)
        
        # Define dimensions
        costmap_size = 50
        goal_size = 2
        
        # ResNet-like CNN for costmap
        self.costmap_cnn = nn.Sequential(
            nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False),  # Initial conv
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
            
            # Residual Blocks
            self._make_residual_block(64, 64, blocks=2),
            self._make_residual_block(64, 128, stride=2, blocks=2),
            self._make_residual_block(128, 256, stride=2, blocks=2),
            
            nn.AdaptiveAvgPool2d((1, 1))  # Global average pooling
        )
        
        # Calculate the output size after CNN
        cnn_output_dim = 256  # Last block outputs 256 channels

        # MLP for goal point
        self.goal_mlp = nn.Sequential(
            nn.Linear(goal_size, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU()
        )
        
        # Combined MLP
        self.combined_mlp = nn.Sequential(
            nn.Linear(cnn_output_dim + 128, features_dim),
            nn.ReLU(),
            nn.Linear(features_dim, features_dim),
            nn.ReLU()
        )
        
        # Initialize weights
        self._initialize_weights()

    def _make_residual_block(self, in_channels, out_channels, stride=1, blocks=2):
        layers = []
        layers.append(ResidualBlock(in_channels, out_channels, stride))
        for _ in range(1, blocks):
            layers.append(ResidualBlock(out_channels, out_channels))
        return nn.Sequential(*layers)

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # Split observations into costmap and goal
        costmap = observations[:, :-2]  # Assuming last 2 are goal
        goal = observations[:, -2:]
        
        # Process costmap
        costmap = costmap.view(-1, 1, 50, 50)  # Reshape to (batch, channels, H, W)
        costmap_features = self.costmap_cnn(costmap)  # Output: (batch, 256, 1, 1)
        costmap_features = costmap_features.view(-1, 256)  # Flatten
        
        # Process goal
        goal_features = self.goal_mlp(goal)  # Output: (batch, 128)
        
        # Combine features
        combined = torch.cat((costmap_features, goal_features), dim=1)  # (batch, 384)
        combined_features = self.combined_mlp(combined)  # (batch, features_dim)
        
        return combined_features

class ResidualBlock(nn.Module):
    """
    A basic residual block for ResNet.
    """

    def __init__(self, in_channels, out_channels, stride=1):
        super(ResidualBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, 
                               kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)
        
        self.conv2 = nn.Conv2d(out_channels, out_channels, 
                               kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)
        
        self.downsample = None
        if stride != 1 or in_channels != out_channels:
            self.downsample = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, 
                          kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(out_channels)
            )
        
    def forward(self, x):
        identity = x
        
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        
        out = self.conv2(out)
        out = self.bn2(out)
        
        if self.downsample is not None:
            identity = self.downsample(x)
        
        out += identity
        out = self.relu(out)
        
        return out
