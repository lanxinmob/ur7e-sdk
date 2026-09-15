import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose_distance(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return (
        np.linalg.norm(a[:3] - b[:3]),
        (Rotation.from_rotvec(a[3:]).inv() * Rotation.from_rotvec(b[3:])).magnitude(),
    )


class PoseTrajectoryInterpolator:
    """Linear translation and shortest-arc SO(3) interpolation, held at endpoints."""

    def __init__(self, times, poses):
        self.times = np.array(times, dtype=float, copy=True)
        self.poses = np.array(poses, dtype=float, copy=True)
        if (
            self.times.ndim != 1
            or len(self.times) == 0
            or self.poses.shape != (len(self.times), 6)
            or not np.isfinite(self.times).all()
            or not np.isfinite(self.poses).all()
            or np.any(np.diff(self.times) <= 0)
        ):
            raise ValueError(
                "Expected finite, strictly increasing times and (T,6) poses"
            )
        self._slerp = (
            Slerp(self.times, Rotation.from_rotvec(self.poses[:, 3:]))
            if len(self.times) > 1
            else None
        )

    def __call__(self, times):
        t = np.asarray(times, dtype=float)
        if not np.isfinite(t).all():
            raise ValueError("Non-finite interpolation time")
        flat = np.clip(t.reshape(-1), self.times[0], self.times[-1])
        out = np.repeat(self.poses[:1], len(flat), axis=0)
        if self._slerp is not None:
            for i in range(3):
                out[:, i] = np.interp(flat, self.times, self.poses[:, i])
            out[:, 3:] = self._slerp(flat).as_rotvec()
        return out.reshape(t.shape + (6,))

    def trim(self, start_t, end_t):
        if start_t > end_t:
            raise ValueError("Reversed interval")
        t = np.unique(
            np.r_[
                start_t,
                self.times[(self.times > start_t) & (self.times < end_t)],
                end_t,
            ]
        )
        return type(self)(t, self(t))

    def drive_to_waypoint(self, pose, time, curr_time, max_pos_speed, max_rot_speed):
        if min(max_pos_speed, max_rot_speed) <= 0:
            raise ValueError("Speeds must be positive")
        start = self(curr_time)
        dp, dr = pose_distance(start, pose)
        end = curr_time + max(time - curr_time, dp / max_pos_speed, dr / max_rot_speed)
        if end == curr_time:
            return type(self)([curr_time], [pose])
        return type(self)([curr_time, end], [start, pose])

    def schedule_waypoint(self, pose, time, curr_time, max_pos_speed, max_rot_speed):
        if time <= curr_time:
            raise ValueError("Waypoint is stale")
        end = max(curr_time, self.times[-1]) if time > self.times[-1] else curr_time
        prefix = self.trim(curr_time, end)
        tail = prefix.drive_to_waypoint(pose, time, end, max_pos_speed, max_rot_speed)
        return type(self)(
            np.r_[prefix.times[:-1], tail.times],
            np.concatenate([prefix.poses[:-1], tail.poses]),
        )
