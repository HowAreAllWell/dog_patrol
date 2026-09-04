import torch
import torch.nn as nn 
from RL2Path.model.custom_cnn import get_model
from skrl.models.torch import Model, GaussianMixin, DeterministicMixin

class ValueNet(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device, output=1, model_version="tcnn_v1", clip_actions=False, goal_include_theta=True, use_gate=True, costmap_width=50):
        Model.__init__(self, observation_space, output, device)
        DeterministicMixin.__init__(self, clip_actions)
        
        CustomCNN = get_model(model_version)
        
        # 值函数用特征提取器（对齐 vf_features_extractor）
        self.vf_features_extractor = CustomCNN(
            observation_space, features_dim=256, 
            goal_include_theta=goal_include_theta, 
            # apply_gate_on_dynamic=use_gate,
            action_space=action_space.shape[0],
            costmap_width=costmap_width,
        )
        
        # 值网络（vf）：256 → 64 → 64
        self.value_net = nn.Sequential(
            nn.Linear(256, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh()
        )
        
        # 值函数输出（单个标量）
        self.value_output = nn.Linear(64, 1)
        
    def compute(self, inputs, role=None):
        # print("compute state shape ", inputs["states"].shape)
        states = inputs["states"].to(self.device)
        
        features = self.vf_features_extractor(states)
        x = self.value_net(features)
        value = self.value_output(x)   # (B,)
        # print("compute value shape ", value.shape)
        return value, {}                     # 或者 (value, {}) 看调用方