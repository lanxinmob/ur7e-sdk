import time

import numpy as np

from ur7e_sdk.control.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from ur7e_sdk.control.rtde_interpolation_controller import (
    STATE_WIDTH,
    RTDEInterpolationController,
)
from ur7e_sdk.runtime.config import load_config
from ur7e_sdk.safety import SafetyError, SafetyLayer


class BimanualUREnv:
    """One or two arms, absolute base-frame TCP actions, Unix timestamps."""

    def __init__(self, config):
        self.config = load_config(config)
        self.n_arms = len(self.config["arms"])
        self.controller = RTDEInterpolationController(self.config)
        self.clock_offset = time.time() - time.monotonic()
        self.cameras = []

    def start(self):
        try:
            if self.config.get("cameras"):
                from ur7e_sdk.hardware.camera import Camera

                for c in self.config["cameras"]:
                    self.cameras.append(Camera(c))
            self.controller.start()
        except BaseException:
            self.controller.stop()
            for camera in self.cameras:
                camera.close()
            raise
        return self

    def stop(self):
        try:
            self.controller.close()
        finally:
            errors = []
            for camera in self.cameras:
                try:
                    camera.close()
                except Exception as exc:
                    errors.append(str(exc))
            self.cameras.clear()
            if errors:
                raise RuntimeError("; ".join(errors))

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        if exc_type is None:
            self.stop()
        else:
            self.controller.stop()
            for camera in self.cameras:
                camera.close()
            self.cameras.clear()

    @property
    def is_ready(self):
        return (
            self.controller.ready.is_set()
            and self.controller.process.is_alive()
            and not self.controller.failed.is_set()
            and not self.controller.stop_event.is_set()
        )

    def get_obs(self, n_obs_steps=1):
        self.controller.check()
        if not isinstance(n_obs_steps, int) or n_obs_steps < 1:
            raise ValueError("n_obs_steps must be a positive integer")
        rows = self.controller.history.get_all()
        if len(rows) == 0:
            raise RuntimeError("No observations yet")
        if time.monotonic() - rows[-1, 0] > min(
            a["safety"]["state_timeout"] for a in self.config["arms"]
        ):
            self.controller.stop_event.set()
            raise SafetyError("Observation stream is stale")
        arm_rows = [
            rows[:, 1 + i * STATE_WIDTH : 1 + (i + 1) * STATE_WIDTH]
            for i in range(self.n_arms)
        ]
        # Align to the oldest latest sensor timestamp; never extrapolate.
        end = min(min(r[-1, 25], r[-1, 26]) for r in arm_rows)
        try:
            camera_data = [c.snapshot() for c in self.cameras]
        except RuntimeError as exc:
            if "history" not in str(exc):
                self.controller.stop_event.set()
            raise
        if camera_data:
            end = min(end, min(t[-1] for t, _ in camera_data))
        times = end - np.arange(n_obs_steps - 1, -1, -1) / self.config["obs_frequency"]
        result = {"timestamp": times + self.clock_offset}
        for i, r in enumerate(arm_rows):
            rt, indices = np.unique(r[:, 25], return_index=True)
            gt, gi = np.unique(r[:, 26], return_index=True)
            if times[0] < max(rt[0], gt[0]):
                raise RuntimeError(
                    "Insufficient observation history; wait before requesting this horizon"
                )
            pose = PoseTrajectoryInterpolator(rt, r[indices, :6])(times)
            result[f"robot{i}_eef_pos"] = pose[:, :3]
            result[f"robot{i}_eef_rot_axis_angle"] = pose[:, 3:]
            result[f"robot{i}_gripper_width"] = np.interp(times, gt, r[gi, 24])[:, None]
            for name, sl in (
                ("joint_pos", slice(6, 12)),
                ("joint_vel", slice(12, 18)),
                ("tcp_speed", slice(18, 24)),
            ):
                result[f"robot{i}_{name}"] = np.stack(
                    [
                        np.interp(times, rt, r[indices, col])
                        for col in range(sl.start, sl.stop)
                    ],
                    axis=1,
                )
            result[f"robot{i}_receive_timestamp"] = np.array(
                [r[-1, 25] + self.clock_offset]
            )
            result[f"robot{i}_gripper_receive_timestamp"] = np.array(
                [r[-1, 26] + self.clock_offset]
            )
        for i, (stamps, frames) in enumerate(camera_data):
            indices = np.searchsorted(stamps, times, side="right") - 1
            if np.any(indices < 0):
                raise RuntimeError("Insufficient camera history")
            if np.max(times - stamps[indices]) > self.config["cameras"][i].get(
                "timeout", 0.5
            ):
                self.controller.stop_event.set()
                raise SafetyError("Camera alignment gap is too large")
            result[f"camera{i}_rgb"] = np.stack([frames[j] for j in indices])
            result[f"camera{i}_timestamp"] = stamps[indices] + self.clock_offset
        return result

    def exec_action(self, actions, timestamps):
        """Replace future plan atomically. Return accepted row count; drop past rows."""
        self.controller.check()
        try:
            a, t = np.asarray(actions, dtype=float), np.asarray(timestamps, dtype=float)
            if a.ndim == 1:
                a = a[None, :]
            if a.ndim == 2 and a.shape[1] == self.n_arms * 7:
                a = a.reshape(-1, self.n_arms, 7)
            t = np.atleast_1d(t)
            if (
                a.shape != (len(t), self.n_arms, 7)
                or t.ndim != 1
                or not np.isfinite(a).all()
                or not np.isfinite(t).all()
                or np.any(np.diff(t) <= 0)
                or len(t) > 256
            ):
                raise SafetyError(
                    "Expected finite (T,N,7) or (T,7*N) actions and increasing (T,) times; T<=256"
                )
            # A fixed offset isolates accepted scheduling from later wall-clock jumps.
            mono = t - self.clock_offset
            mask = mono > time.monotonic() + 2 / self.config["frequency"]
            a, mono = a[mask], mono[mask]
            if not len(mono):
                return 0
            if mono[-1] - time.monotonic() > min(
                arm["safety"]["max_horizon"] for arm in self.config["arms"]
            ):
                raise SafetyError("Action horizon too long")
            for i, arm in enumerate(self.config["arms"]):
                for action in a[:, i]:
                    SafetyLayer(arm["safety"]).target(action)
            self.controller.submit(a, mono)
            return len(mono)
        except (ValueError, TypeError, SafetyError):
            self.controller.stop_event.set()
            raise

    exec_actions = exec_action

    def action_from_obs(self, obs=None):
        obs = self.get_obs() if obs is None else obs
        return np.stack(
            [
                np.r_[
                    obs[f"robot{i}_eef_pos"][-1],
                    obs[f"robot{i}_eef_rot_axis_angle"][-1],
                    obs[f"robot{i}_gripper_width"][-1],
                ]
                for i in range(self.n_arms)
            ]
        )
