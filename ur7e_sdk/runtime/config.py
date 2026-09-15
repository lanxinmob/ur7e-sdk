from copy import deepcopy
from pathlib import Path

import numpy as np
import yaml

SAFETY = dict(
    workspace_min=[0.15, -0.5, 0.1],
    workspace_max=[0.8, 0.5, 0.8],
    max_pos_speed=0.1,
    max_rot_speed=0.5,
    max_gripper_speed=0.1,
    max_joint_speed=1.0,
    joint_min=[-6.28] * 6,
    joint_max=[6.28] * 6,
    max_tracking_pos=0.05,
    max_tracking_rot=0.3,
    state_timeout=0.5,
    command_timeout=1.0,
    max_horizon=2.0,
    max_cycle_lag=0.1,
)


def load_config(source, require_commissioned=True):
    if isinstance(source, (str, Path)):
        with open(source) as f:
            source = yaml.safe_load(f)
    c = deepcopy(source)
    if not isinstance(c, dict):
        raise ValueError("Configuration must be a mapping")
    c.setdefault("backend", "sim")
    c.setdefault("frequency", 125)
    c.setdefault("obs_frequency", 20)
    c.setdefault("history", 512)
    c.setdefault("lookahead_time", 0.1)
    c.setdefault("gain", 300)
    if c["backend"] not in ("sim", "ur"):
        raise ValueError("backend must be sim or ur")
    if not 1 <= c["frequency"] <= 500 or not 0 < c["obs_frequency"] <= c["frequency"]:
        raise ValueError("Invalid control/observation frequency")
    if not 0.03 <= c["lookahead_time"] <= 0.2 or not 100 <= c["gain"] <= 2000:
        raise ValueError("Invalid RTDE servo parameters")
    if not isinstance(c["history"], int) or c["history"] < 2:
        raise ValueError("history must be an integer >= 2")
    arms = c.get("arms", [])
    if not 1 <= len(arms) <= 2:
        raise ValueError("Configure one or two arms")
    ips = []
    for arm in arms:
        arm.setdefault("initial_pose", [0.4, 0, 0.3, 0, 3.141592653589793, 0])
        unknown = arm.get("safety", {}).keys() - SAFETY.keys()
        if unknown:
            raise ValueError(f"Unknown safety keys: {sorted(unknown)}")
        arm["safety"] = SAFETY | arm.get("safety", {})
        s = arm["safety"]
        for key in ("workspace_min", "workspace_max", "joint_min", "joint_max"):
            n = 3 if key.startswith("workspace") else 6
            a = np.asarray(s[key], dtype=float)
            if a.shape != (n,) or not np.isfinite(a).all():
                raise ValueError(f"Invalid safety.{key}")
            s[key] = a.tolist()
        if np.any(np.array(s["workspace_min"]) >= s["workspace_max"]) or np.any(
            np.array(s["joint_min"]) >= s["joint_max"]
        ):
            raise ValueError("Safety bounds must increase")
        for key in SAFETY.keys() - {
            "workspace_min",
            "workspace_max",
            "joint_min",
            "joint_max",
        }:
            if not np.isfinite(s[key]) or s[key] <= 0:
                raise ValueError(f"Invalid safety.{key}")
        if c["backend"] == "ur":
            if not arm.get("ip"):
                raise ValueError("Robot ip is required")
            ips.append(arm["ip"])
        if c["backend"] == "ur" and require_commissioned:
            if arm.get("commissioned") is not True:
                raise ValueError(
                    "Set ip and commissioned: true after checking TCP/payload/safety"
                )
            for key, n in (("tcp", 6), ("payload_cog", 3)):
                a = np.asarray(arm.get(key), dtype=float)
                if a.shape != (n,) or not np.isfinite(a).all():
                    raise ValueError(f"Explicit {key} required")
            if not 0 <= arm.get("payload_mass", -1) <= 7.5:
                raise ValueError("Explicit payload_mass in [0,7.5] kg required")
        g = arm.setdefault("gripper", {})
        defaults = dict(
            port=63352, open_raw=0, closed_raw=255, speed=64, force=20, timeout=0.15
        )
        arm["gripper"] = defaults | g
        g = arm["gripper"]
        if not 0 <= g["open_raw"] < g["closed_raw"] <= 255:
            raise ValueError("Invalid Robotiq calibration")
        for key in ("speed", "force"):
            if not isinstance(g[key], int) or not 0 <= g[key] <= 255:
                raise ValueError(f"Invalid gripper {key}")
        if not np.isfinite(g["timeout"]) or not 0 < g["timeout"] <= s["state_timeout"]:
            raise ValueError("Invalid gripper timeout")
    if len(ips) != len(set(ips)):
        raise ValueError("Duplicate robot IPs")
    for camera in c.get("cameras", []):
        for key, default in (
            ("history", 128),
            ("width", 640),
            ("height", 480),
            ("fps", 30),
        ):
            value = camera.get(key, default)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"Invalid camera {key}")
        if (
            not np.isfinite(camera.get("timeout", 0.5))
            or camera.get("timeout", 0.5) <= 0
        ):
            raise ValueError("Invalid camera timeout")
    return c
