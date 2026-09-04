import torch
import torch.nn as nn 
from RL2Path.model.custom_cnn import get_model
from skrl.models.torch import Model, GaussianMixin

class PolicyNet(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, model_version="tcnn_v1", clip_actions=False, goal_include_theta=True, use_gate=True, costmap_width=50):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions)
        
        # self.features_extractor = CustomCNN(observation_space, features_dim=256)

        CustomCNN = get_model(model_version)

        # 策略用特征提取器（对齐 pi_features_extractor）
        self.pi_features_extractor = CustomCNN(
            observation_space, features_dim=256, 
            goal_include_theta=goal_include_theta, 
            # apply_gate_on_dynamic=use_gate,
            action_space=action_space.shape[0],
            costmap_width=costmap_width,
        )

        # 策略网络（pi）：256 → 64 → 64
        self.policy_net = nn.Sequential(
            nn.Linear(256, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh()
        )

        # 策略输出（动作）
        self.action_net = nn.Linear(64, self.num_actions)
        
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))

    def compute(self, inputs, role=None):
        states = inputs["states"].to(self.device)

        features = self.pi_features_extractor(states)
        x = self.policy_net(features)
        mean = self.action_net(x)

        return mean, self.log_std_parameter, {}