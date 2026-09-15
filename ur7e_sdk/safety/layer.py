import numpy as np

from ur7e_sdk.control.pose_trajectory_interpolator import pose_distance


class SafetyError(RuntimeError):
    pass


class SafetyLayer:
    """Independent fail-closed validation; the process latches all violations."""

    def __init__(self, config):
        self.config = config

    def target(self, action):
        a = np.asarray(action, dtype=float)
        s = self.config
        if a.shape != (7,) or not np.isfinite(a).all():
            raise SafetyError("Action must contain seven finite values")
        if np.any(a[:3] < s["workspace_min"]) or np.any(a[:3] > s["workspace_max"]):
            raise SafetyError("TCP outside configured workspace")
        if not 0 <= a[6] <= 0.085:
            raise SafetyError("Gripper width outside [0, 0.085] m")

    def segment(self, start, end, duration):
        self.target(start)
        self.target(end)
        if not np.isfinite(duration) or duration <= 0:
            raise SafetyError("Nonpositive segment duration")
        dp, dr = pose_distance(start[:6], end[:6])
        s = self.config
        violations = []
        for name, delta, limit, unit in (
            ("translation", dp, s["max_pos_speed"], "m"),
            ("rotation", dr, s["max_rot_speed"], "rad"),
            ("gripper", abs(end[6] - start[6]), s["max_gripper_speed"], "m"),
        ):
            if delta > limit * duration + 1e-9:
                violations.append(
                    f"{name}: delta={delta:.6f} {unit}, "
                    f"required={delta / duration:.4f} {unit}/s "
                    f"> limit={limit:.4f} {unit}/s"
                )
        if violations:
            raise SafetyError(
                f"Waypoint deadline exceeds configured speed limits "
                f"(available={duration:.4f}s): " + "; ".join(violations)
            )

    def joint_target(self, joints):
        q = np.asarray(joints, dtype=float)
        if (
            q.shape != (6,)
            or not np.isfinite(q).all()
            or np.any(q < self.config["joint_min"])
            or np.any(q > self.config["joint_max"])
        ):
            raise SafetyError("Joint target outside configured limits")

    def state(self, state, target, now):
        s = self.config
        for key in (
            "pose",
            "q",
            "qd",
            "speed",
            "width",
            "timestamp",
            "gripper_timestamp",
        ):
            if not np.isfinite(state[key]).all():
                raise SafetyError(f"Non-finite state: {key}")
        self.target(np.r_[state["pose"], state["width"]])
        if (
            not state["safe"]
            or now - state["timestamp"] > s["state_timeout"]
            or now - state["gripper_timestamp"] > s["state_timeout"]
        ):
            raise SafetyError("Unsafe or stale hardware state")
        if (
            np.any(state["q"] < s["joint_min"])
            or np.any(state["q"] > s["joint_max"])
            or np.max(np.abs(state["qd"])) > s["max_joint_speed"]
        ):
            raise SafetyError("Joint limit exceeded")
        dp, dr = pose_distance(state["pose"], target[:6])
        if dp > s["max_tracking_pos"] or dr > s["max_tracking_rot"]:
            raise SafetyError("TCP tracking error exceeded")
        if (
            np.linalg.norm(state["speed"][:3]) > s["max_pos_speed"] * 1.25
            or np.linalg.norm(state["speed"][3:]) > s["max_rot_speed"] * 1.25
        ):
            raise SafetyError("Measured TCP speed exceeded")
