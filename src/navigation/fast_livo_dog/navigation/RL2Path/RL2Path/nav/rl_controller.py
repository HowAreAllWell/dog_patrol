"""load_skrl_agent.py
=======================
Utility to **load a trained skrl agent** (weights + hyper‑parameters) from a YAML
configuration file *and* a checkpoint, then run it inside **any** Gym‑like
environment you supply – no need to recreate the original training env.

Typical usage
-------------
```python
import gymnasium as gym
from load_skrl_agent import AgentInterface

# Your own environment – can differ from the one used for training
env = gym.make("Pendulum-v1")

agent = AgentInterface(
    env=env,
    cfg_path="configs/ppo_quadcopter.yaml",   # path to YAML shown in the prompt
    checkpoint_path="weights/ckpt_1200.pt"    # .pt or .pth saved by skrl
)

obs, _ = env.reset()
for _ in range(1000):
    action = agent.act(obs)   # <-- single call per step
    obs, rew, term, trunc, info = env.step(action)
    if term or trunc:
        agent.reset_timestep()
        obs, _ = env.reset()
```

The file keeps dependencies minimal – only **PyYAML** and **skrl** (plus
PyTorch/JAX depending on the checkpoint). The full API is documented inside the
class docstring.
"""

from __future__ import annotations

import os
import warnings
from typing import Any, Dict, Union

import yaml  # PyYAML
import torch

import skrl
from packaging import version

# skrl components -------------------------------------------------------------
from skrl.memories.torch import RandomMemory  # type: ignore
from skrl.agents.torch.ppo import PPO  # type: ignore
# from skrl.utils.model_instantiators import instantiate_models  # type: ignore

# -----------------------------------------------------------------------------
# Version guard
# -----------------------------------------------------------------------------
_MIN_SKRL_VERSION = "1.4.2"
if version.parse(skrl.__version__) < version.parse(_MIN_SKRL_VERSION):
    raise RuntimeError(
        f"skrl>={_MIN_SKRL_VERSION} required, but found {skrl.__version__}. "
        "Upgrade with `pip install --upgrade skrl`"
    )

# -----------------------------------------------------------------------------
# Public interface
# -----------------------------------------------------------------------------

class RLControllerInterface:  # pylint: disable=too-few-public-methods
    """Load a trained skrl PPO agent **without needing the original environment**.

    You simply tell me the *dimensions* (or full Gym spaces) of observations
    and actions – that is all the information required to re‑instantiate the
    networks and load their parameters.

    Parameters
    ----------
    cfg_path : str
        Path to the YAML config used during training – we read the ``models``
        section to rebuild the policy / value nets and the ``agent`` section for
        hyper‑parameters.
    checkpoint_path : str
        Path to the ``.pt/.pth`` file with the saved weights.
    obs_space : gym.spaces.Space | int
        Either a Gymnasium observation space *or* an **integer** giving the
        observation dimension. If an int is supplied, we build
        ``gym.spaces.Box(-inf, inf, (obs_dim,), dtype=float32)``.
    act_space : gym.spaces.Space | int
        Action space (same semantics as ``obs_space``). An int will generate an
        unbounded continuous ``Box`` of that dimension.
    device : str | torch.device, default "cpu"
        Torch device for the networks.

    Notes
    -----
    *Only PPO (Torch backend) is implemented right now.* Extending to IPPO,
    MAPPO, etc. just requires wiring the correct agent class.
    """

    def __init__(
        self,
        *,
        cfg_path: str,
        checkpoint_path: str,
        obs_space: "gym.spaces.Space | int",
        act_space: "gym.spaces.Space | int",
        device: Union[str, torch.device] | None = None,
    ) -> None:
        import gymnasium as gym  # local import
        import numpy as np

        # ------------------------------------------------------------------
        # 0. Build (or accept) Gym spaces
        # ------------------------------------------------------------------
        if isinstance(obs_space, int):
            obs_space = gym.spaces.Box(-np.inf, np.inf, (obs_space,), dtype=np.float32)
        if isinstance(act_space, int):
            act_space = gym.spaces.Box(-np.inf, np.inf, (act_space,), dtype=np.float32)
        if not isinstance(obs_space, gym.Space) or not isinstance(act_space, gym.Space):
            raise TypeError("obs_space / act_space must be gym spaces or integers")

        self.observation_space = obs_space  # type: ignore[assignment]
        self.action_space = act_space  # type: ignore[assignment]

        self.device = torch.device(device or ("cuda:0" if torch.cuda.is_available() else "cpu"))

        # ------------------------------------------------------------------
        # 1. Load YAML config for models / memory / agent hyper‑params
        # ------------------------------------------------------------------
        with open(cfg_path, "r", encoding="utf-8") as fh:
            cfg: Dict[str, Any] = yaml.safe_load(fh)

        if "models" not in cfg or "agent" not in cfg:
            raise ValueError("YAML config must contain at least 'models' and 'agent' sections")

        models_cfg = cfg["models"]
        agent_cfg = cfg["agent"]

        # Allow safe fallbacks if some keys missing (common when pruning configs)
        agent_cfg.setdefault("learning_rate_scheduler", None)
        agent_cfg.setdefault("value_preprocessor", None)
        agent_cfg.setdefault("state_preprocessor", None)

        # ------------------------------------------------------------------
        # 2. Instantiate policy / value networks according to config
        # ------------------------------------------------------------------
        from skrl.utils.model_instantiators.torch import (
            gaussian_model,
            deterministic_model,
            shared_model,
        )

        # print(self.observation_space)
        # print(self.action_space)

        if models_cfg.get("separate", False):
            # Two independent networks
            self.models = {
                "policy": gaussian_model(
                    observation_space=self.observation_space,
                    action_space=self.action_space,
                    device=self.device,
                    **models_cfg["policy"],
                ),
                "value": deterministic_model(
                    observation_space=self.observation_space,
                    action_space=self.action_space,
                    device=self.device,
                    **models_cfg["value"],
                ),
            }
        else:
            # Shared backbone - two heads (roles order matters!)
            self.models = {
                "shared": shared_model(
                    observation_space=self.observation_space,
                    action_space=self.action_space,
                    device=self.device,
                    roles=["policy", "value"],
                    parameters=[models_cfg["policy"], models_cfg["value"]],
                    single_forward_pass=True
                )
            }

        # ------------------------------------------------------------------
        # 3. Create dummy rollout memory (required by skrl agents)
        # ------------------------------------------------------------------ Create dummy rollout memory (required by skrl agents) Create dummy rollout memory (required by skrl agents)
        # ------------------------------------------------------------------
        rollouts = agent_cfg.get("rollouts", 1)
        self.memory = RandomMemory(
            memory_size=-1
        )

        # ------------------------------------------------------------------
        # 4. Instantiate PPO agent and load checkpoint
        # ------------------------------------------------------------------
        
        # print(self.models)
        
        # print(self.models.get("policy", None))
        # print(self.models.get("value", None))
        
        # print(self.models)
        
        self.agent = PPO(
            models=self.models,
            memory=self.memory,
            observation_space=self.observation_space,  # type: ignore[arg-type]
            action_space=self.action_space,  # type: ignore[arg-type]
            device=self.device,
            cfg=agent_cfg,
        )

        checkpoint_path = os.path.abspath(checkpoint_path)
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(checkpoint_path)

        print(f"[INFO] Loading checkpoint: {checkpoint_path}")
        self.agent.load(checkpoint_path)
        self.agent.set_running_mode("eval")  # inference mode

        self._timestep = 0  # for algorithms that expect it

    # ------------------------------------------------------------------
    # Inference API
    # ------------------------------------------------------------------
    def act(self, obs: Any) -> Any:  # noqa: D401
        """Return an action for **one** observation array or tensor."""
        if isinstance(obs, dict):
            obs = next(v for v in obs.values() if _is_array_like(v))

        obs_tensor = torch.as_tensor(obs, device=self.device)
        if obs_tensor.ndim == 1:
            obs_tensor = obs_tensor.unsqueeze(0)

        with torch.inference_mode():
            outputs = self.agent.act(obs_tensor, timestep=self._timestep, timesteps=self._timestep)  # type: ignore[arg-type]
        self._timestep += 1

        action = outputs[-1].get("mean_actions", outputs[0])
        return action

    def reset_timestep(self):
        """Call after every environment reset (if needed)."""
        self._timestep = 0
        
    # ------------------------------------------------------------------
    # 🔥 NEW: Pure‑pursuit helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _first_true_index(mask: torch.Tensor) -> torch.Tensor:
        """Utility: return index of first *True* in each row (or last index if none)."""
        # mask: (B, T) boolean tensor on device
        # Convert False to large value so argmin picks first True, else last
        B, T = mask.shape
        inf = T  # any value > max index
        idx = torch.where(mask, torch.arange(T, device=mask.device).expand(B, -1), inf)
        first = idx.min(dim=1).values
        # Clamp: if "no True" (value==inf) → last element (T-1)
        first = torch.where(first == inf, torch.full_like(first, T - 1), first)
        return first

    def pure_pursuit_target(
        self,
        paths: torch.Tensor,
        positions: torch.Tensor,
        lookahead: float,
    ) -> torch.Tensor:
        """Compute look‑ahead targets using classic pure pursuit.

        Parameters
        ----------
        paths : torch.Tensor
            Shape ``(B, T, D)`` – batch of waypoint sequences.
        positions : torch.Tensor
            Shape ``(B, D)`` – current agent positions.
        lookahead : float
            Positive look‑ahead distance (in the same units as the path).

        Returns
        -------
        torch.Tensor
            Shape ``(B, D)`` – selected target points for each agent.
        """
        if paths.ndim != 3:
            raise ValueError("paths must be (batch, time, dim)")
        if positions.ndim != 2 or positions.shape[0] != paths.shape[0]:
            raise ValueError("positions must be (batch, dim) with same batch size as paths")

        # Ensure tensors live on the same device as the policy
        paths = paths.to(self.device)
        positions = positions.to(self.device)

        # print(f"paths shape: {paths.shape}")
        # print(f"positions shape: {positions.shape}")

        dists = torch.norm(paths - positions.unsqueeze(1), dim=-1)  # (B, T)
        mask = dists >= lookahead  # first waypoint satisfying look‑ahead

        first_valid = self._first_true_index(mask)
        targets = paths[torch.arange(paths.size(0), device=self.device), first_valid]
        return targets

    def act_pure_pursuit(
        self,
        observation_wo_target: Union[torch.Tensor, Any],
        paths: Union[torch.Tensor, Any],
        positions: Union[torch.Tensor, Any],
        lookahead: float = 1.0,
    ) -> Tuple[Any, torch.Tensor]:
        """Compute pure‑pursuit targets **and** query the policy in one call.

        This helper handles the full control loop:

        1. Computes a target point for each agent via pure pursuit.
        2. Builds a minimal observation for the RL policy – either the *relative*
           vector to the target (default) or the concatenation of agent position
           and absolute target coordinates (``concat_abs=True``).
        3. Calls :meth:`act` and returns both the actions and the selected targets.

        The observation layout must match what the loaded policy expects. The
        relative‑vector variant is often sufficient: if your state already
        contained agent position/orientation, replace that part with
        ``target - position``. Otherwise, use ``concat_abs`` for quick
        prototyping.
        """
        # To tensor – tolerates lists/NumPy
        paths_t = torch.as_tensor(paths, device=self.device)
        pos_t = torch.as_tensor(positions, device=self.device)

        # print(paths.shape)

        targets = self.pure_pursuit_target(paths_t, pos_t, lookahead)

        # if concat_abs:
        #     obs = torch.cat([pos_t, targets], dim=-1)
        # else:
        #     obs = targets - pos_t  # relative vector
        
        # print(observation_wo_target.shape)
        # print(targets.shape)
        
        obs = torch.cat(
            [
                observation_wo_target,  # whatever the agent expects
                targets - pos_t  # relative vector to target
            ],
            dim=-1
        )

        actions = self.act(obs)
        return actions, targets


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def _is_array_like(x):  # minimal duck‑typing helper
    return hasattr(x, "shape") or isinstance(x, (list, tuple))


if __name__ == "__main__":
    agent = RLControllerInterface(
        cfg_path="checkpoints/drone/skrl_ppo_cfg.yaml",
        checkpoint_path="checkpoints/drone/best_agent.pt",
        obs_space=12,      
        act_space=4
    )
    
    obs = torch.rand(32, 12)
    outputs = agent.act(obs)
    
    print(outputs.shape)

    