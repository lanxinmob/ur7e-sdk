import importlib

import numpy as np


def load_policy(spec):
    """Import a trusted module:factory; factory() returns callable policy(obs)."""
    module, separator, factory = spec.partition(":")
    if not separator:
        raise ValueError("Policy must be module:factory")
    policy = getattr(importlib.import_module(module), factory)()
    if not callable(policy):
        raise TypeError("Policy factory must return a callable")
    return policy


class HoldPolicy:
    def __call__(self, obs):
        n = len([k for k in obs if k.endswith("_eef_pos")])
        action = np.stack(
            [
                np.r_[
                    obs[f"robot{i}_eef_pos"][-1],
                    obs[f"robot{i}_eef_rot_axis_angle"][-1],
                    obs[f"robot{i}_gripper_width"][-1],
                ]
                for i in range(n)
            ]
        )
        return action[None]


def schedule_output(output, obs, action_dt, latency):
    """Implicit deadlines are anchored to observation time, including inference delay."""
    if isinstance(output, dict):
        actions = np.asarray(output["actions"], dtype=float)
        if "timestamps" in output:
            return actions, np.asarray(output["timestamps"], dtype=float)
    else:
        actions = np.asarray(output, dtype=float)
    if actions.ndim not in (2, 3) or len(actions) == 0:
        raise ValueError(
            "Policy output must be (T,7*N), (T,N,7), or a timestamped dict"
        )
    times = obs["timestamp"][-1] + latency + np.arange(len(actions)) * action_dt
    return actions, times
