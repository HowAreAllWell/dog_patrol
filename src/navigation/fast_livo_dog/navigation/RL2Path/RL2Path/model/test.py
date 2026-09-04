# import argparse

# from isaaclab.app import AppLauncher

# # add argparse arguments
# parser = argparse.ArgumentParser(
#     description="This script demonstrates adding a custom robot to an Isaac Lab environment."
# )
# parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# # append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# # parse the arguments
# args_cli = parser.parse_args()

# # launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app

from collections import defaultdict, OrderedDict
from stable_baselines3 import PPO
from gymnasium.spaces import Box
from RL2Path.model.custom_tcnn_v0 import CustomCNN
from skrl.models.torch import Model, DeterministicMixin, GaussianMixin
import torch.nn as nn
import numpy as np
import torch

from skrl.models.torch import Model, DeterministicMixin
import torch.nn as nn
import pprint, json, pathlib, re

from RL2Path.model.policy import PolicyNet
from RL2Path.model.value import ValueNet

model = PPO.load("outputs/ckpts/tcnn_v1/model_7.zip", device="cuda:1")

# print(model.policy.state_dict().keys())

observation_space = Box(low=0, high=1, shape=(31253,), dtype=np.float32)
action_space = Box(low=0.0, high=1.0, shape=(10,), dtype=np.float32)

# custom_model = CustomPolicy(observation_space=observation_space, action_space=action_space, device="cuda")

policy_model = PolicyNet(observation_space=observation_space, action_space=action_space, device="cuda:1", goal_include_theta=False, model_version="tcnn_v1")
value_model = ValueNet(observation_space=observation_space, action_space=action_space, device="cuda:1", goal_include_theta=False, model_version="tcnn_v1")

def get_name_map(skrl_model, sb3_model):
    sd_dst = skrl_model.state_dict()
    sd_src = sb3_model.policy.state_dict()
    
    name_map = OrderedDict()

    # ---------- 1. 先做“完全相同名字”的配对 ----------
    for k in sd_dst.keys():
        if k in sd_src:
            name_map[k] = k

    # ---------- 2. 用前缀规则补齐 ----------
    prefix_rules = [
        # a) SB3 的 log_std → skrl 的 log_std_parameter
        (r"^log_std$",                     "log_std_parameter"),
        # b) mlp_extractor.policy_net.* → policy_net.*
        (r"^mlp_extractor\.policy_net\.",  "policy_net."),
        # c) mlp_extractor.value_net.*  → value_net.*
        (r"^mlp_extractor\.value_net\.",   "value_net."),
        # d) SB3 最后的 value_net.*     → skrl 的 value_output.*
        (r"^value_net\.",                  "value_output.")
    ]

    for src_key in sd_src.keys():
        for pat_src, repl_dst in prefix_rules:
            if re.match(pat_src, src_key):
                dst_key = re.sub(pat_src, repl_dst, src_key)
                # 避免重复覆盖，且确保目标模型里确实有这个键
                if dst_key in sd_dst and dst_key not in name_map:
                    name_map[dst_key] = src_key

    return name_map

policy_name_map = get_name_map(policy_model, model)
policy_model.migrate(path="outputs/ckpts/tcnn_v1/model_7.zip", name_map=policy_name_map, verbose=False)

value_name_map = get_name_map(value_model, model)
value_model.migrate(path="outputs/ckpts/tcnn_v1/model_7.zip", name_map=value_name_map, verbose=False)

policy_model.to("cuda:1").eval()
model.policy.to("cuda:1").eval()
value_model.to("cuda:1").eval()

# ========= 3. Sanity check =========
with torch.no_grad():
    dummy = torch.rand(1, *observation_space.shape, device="cuda:1")
    
    # SB3 → 动作均值
    sb3_mean, value, log_prob = model.policy.forward(dummy, deterministic=True)       # forward() 返回 (mean, log_std)
    
    print("sb3 mean: ", sb3_mean)

    # skrl → 动作均值（直接 compute，返回 mean, log_std_parameter, info）
    skrl_mean, log_prob_skrl, _ = policy_model.compute({"states": dummy})
    value_skrl, _ = value_model.compute({"states": dummy})
    
    print(skrl_mean.shape)
    print(log_prob_skrl.shape)
    print(value_skrl.shape)
    
    
    print("skrl mean: ", skrl_mean)

    diff = (sb3_mean - skrl_mean).abs().max().item()
    print(f"max |diff| = {diff:.3e}")
    
    value_diff = (value - value_skrl).abs().max().item()
    print(f"max value diff = {value_diff:.3e}")
    
    values, _, _ = value_model.act({"states": dummy}, role="value")
    
    print("skrl value: ", values.shape)

# sd_src  = model.policy.state_dict()      # SB3 源
# sd_dst = policy_model.state_dict()      # skrl 目标

# name_map = OrderedDict()

# # ---------- 1. 先做“完全相同名字”的配对 ----------
# for k in sd_dst.keys():
#     if k in sd_src:
#         name_map[k] = k

# # ---------- 2. 用前缀规则补齐 ----------
# prefix_rules = [
#     # a) SB3 的 log_std → skrl 的 log_std_parameter
#     (r"^log_std$",                     "log_std_parameter"),
#     # b) mlp_extractor.policy_net.* → policy_net.*
#     (r"^mlp_extractor\.policy_net\.",  "policy_net."),
#     # c) mlp_extractor.value_net.*  → value_net.*
#     (r"^mlp_extractor\.value_net\.",   "value_net."),
#     # d) SB3 最后的 value_net.*     → skrl 的 value_output.*
#     (r"^value_net\.",                  "value_output.")
# ]

# for src_key in sd_src.keys():
#     for pat_src, repl_dst in prefix_rules:
#         if re.match(pat_src, src_key):
#             dst_key = re.sub(pat_src, repl_dst, src_key)
#             # 避免重复覆盖，且确保目标模型里确实有这个键
#             if dst_key in sd_dst and dst_key not in name_map:
#                 name_map[dst_key] = src_key

# ---------- 3. 报告未覆盖的 dst 键 ----------
# unmapped = [k for k in sd_dst if k not in name_map]
# print(f"Still unmapped: {len(unmapped)}")
# for k in unmapped[:30]:
#     print("  ", k)
    
# print(name_map)
    
# 如果 len(unmapped)==0 恭喜你，已经全部搞定

# 可选 verbose=True 打印详细迁移日志
# custom_model.migrate(path="logs/sb3/Template-Rl2path-Direct-v0/2025-07-06_23-33-31/model.zip", name_map=name_map, verbose=False)

# custom_model.to("cuda:1").eval()
# model.policy.to("cuda:1").eval()

# # ========= 3. Sanity check =========
# with torch.no_grad():
#     dummy = torch.rand(1, *observation_space.shape, device="cuda:1")
    
#     # SB3 → 动作均值
#     sb3_mean, value, log_prob = model.policy.forward(dummy, deterministic=True)       # forward() 返回 (mean, log_std)
    
#     print("sb3 mean: ", sb3_mean)

#     # skrl → 动作均值（直接 compute，返回 mean, log_std_parameter, info）
#     skrl_mean, _, _ = custom_model.compute({"states": dummy}, role="policy")
    
#     print("skrl mean: ", skrl_mean)

#     diff = (sb3_mean - skrl_mean).abs().max().item()
#     print(f"max |diff| = {diff:.3e}")
    
# custom_model.save("custom_model.pth")
