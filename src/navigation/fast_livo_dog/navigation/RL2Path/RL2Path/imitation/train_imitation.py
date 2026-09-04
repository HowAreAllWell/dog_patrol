import argparse

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--config", type=str, default="source/RL2Path/config/imitation.toml", help="Path to the configuration file.")
args_cli = parser.parse_args()

import sys
import tomli
from pathlib import Path
import numpy as np
import torch
import gymnasium as gym
import h5py
import pickle
from tqdm import tqdm

from gymnasium import spaces
from stable_baselines3 import PPO
from imitation.data import rollout
from imitation.data.types import Trajectory
from imitation.algorithms.bc import BC
from imitation.util import logger as imit_logger

from RL2Path.model.custom_cnn import get_model


# ─────────────────────────────────────────────
# 从 HDF5 文件加载数据并转换为 Trajectories
# ─────────────────────────────────────────────
def load_transitions_from_h5(path: str, action_dim, batch_size: int = 1000):
    with h5py.File(path, "r") as f:
        observations = np.array(f["observations"])
        actions = np.array(f["actions"])

    print(f"[info] loading {len(observations)} transitions from {path}")

    if len(actions.shape) > 2:
        actions = actions.reshape(actions.shape[0], -1)

    assert actions.shape[1] == action_dim

    trajectories = []
    n = len(observations)

    for start in tqdm(range(0, n, batch_size), desc="Loading Trajectories"):
        end = min(start + batch_size, n)
        for i in range(start, end):
            obs = observations[i]
            new_obs = np.concatenate([
                np.tile(obs[:2500], 3),
                obs[2500:]
            ]).astype(np.float32)
            act = actions[i]
            traj = Trajectory(
                obs=np.array([new_obs, new_obs], dtype=np.float32),  # dummy: duplicated to fit interface
                acts=np.array([act], dtype=np.float32),
                infos=None,
                terminal=np.array([True], dtype=np.bool_)
            )
            trajectories.append(traj)

    return trajectories


# ─────────────────────────────────────────────
# 初始化环境、模型、行为克隆器
# ─────────────────────────────────────────────
def main():
    args = parser.parse_args()
    with open(args.config, 'rb') as f:
        config = tomli.load(f)
    
    imitation_config = config.get('imitation', {})
    ACTION_DIM = config['process']['action_dim']

    TRANSITION_H5_PATH = config['paths']['output']
    CHECKPOINT_DIR = imitation_config.get('checkpoint_dir', './outputs/checkpoints')
    LOG_DIR = imitation_config.get('log_dir', './outputs/logs')
    RESUME_MODEL_PATH = imitation_config.get('resume_model_path', None)
    
    SEED = imitation_config.get('seed', 42)
    BATCH_SIZE = imitation_config.get('batch_size', 512)
    DEVICE = imitation_config.get('device', 'cuda' if torch.cuda.is_available() else 'cpu')
    RESUME = imitation_config.get('resume', False)
    MODEL = imitation_config.get('model', 'custom_cnn')
    FEATURE_DIM = imitation_config.get('feature_dim', 256)
    
    goal_include_theta = imitation_config.get('goal_include_theta', True)

    rng = np.random.default_rng(SEED)
    
    bc_logger = imit_logger.configure(
        folder=LOG_DIR,
        format_strs=["tensorboard"]  # 想要什么输出就写什么
    )
    # 加载数据
    trajectories = load_transitions_from_h5(TRANSITION_H5_PATH, ACTION_DIM)
    
    transitions = rollout.flatten_trajectories(trajectories)
    
    # ─────────────────────────────────────────────
    # 定义训练环境
    # ─────────────────────────────────────────────
    class PathFlowEnv(gym.Env):
        def __init__(self):
            super().__init__()
            self.action_space = spaces.Box(low=0, high=1, shape=(ACTION_DIM,), dtype=np.float32)
            self.observation_space = spaces.Box(low=0, high=1, shape=(7502,), dtype=np.float32)

    # 初始化环境和模型
    env = PathFlowEnv()
    policy_kwargs = dict(
        features_extractor_class=get_model(MODEL),
        features_extractor_kwargs=dict(features_dim=FEATURE_DIM, goal_include_theta=goal_include_theta),
    )
    
    if not RESUME:
        model = PPO(
            "MlpPolicy", 
            env, 
            policy_kwargs=policy_kwargs,
            verbose=1, 
            device=DEVICE,
            batch_size=2048,
        )
    else:
        model = PPO.load(RESUME_MODEL_PATH, env=env, device=DEVICE)
        resume_epoch = int(RESUME_MODEL_PATH.split('_')[-1]) if '_' in RESUME_MODEL_PATH else 0

    # 初始化行为克隆
    bc_trainer = BC(
        rng=rng,
        observation_space=env.observation_space,
        action_space=env.action_space,
        policy=model.policy,
        demonstrations=transitions,
        batch_size=BATCH_SIZE,
        device=DEVICE,
        custom_logger=bc_logger,
    )

    # 训练与保存模型
    if not RESUME:
        current_epoch = 0
    else:
        current_epoch = resume_epoch
    for i in range(7):
        bc_trainer.train(n_epochs=1)
        current_epoch += 1
        model.save(f"{CHECKPOINT_DIR}/model_{current_epoch}")
        print(f"[INFO] Model checkpoint saved: model_{current_epoch}")

if __name__ == "__main__":
    main()
