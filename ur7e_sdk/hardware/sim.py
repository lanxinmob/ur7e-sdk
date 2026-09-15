import time

import numpy as np


class SimRobot:
    def __init__(self, arm, config):
        self.pose = np.array(arm["initial_pose"], dtype=float)
        self.width = 0.085
        self.speed = np.zeros(6)
        self.dt = 1 / config["frequency"]
        self.last_servo = time.monotonic()

    def start(self):
        pass

    def read(self):
        return dict(
            pose=self.pose.copy(),
            q=np.zeros(6),
            qd=np.zeros(6),
            speed=self.speed.copy(),
            width=self.width,
            safe=True,
            timestamp=time.monotonic(),
            gripper_timestamp=time.monotonic(),
        )

    def servo(self, action):
        from scipy.spatial.transform import Rotation

        now = time.monotonic()
        dt = max(now - self.last_servo, self.dt / 2)
        self.last_servo = now
        self.speed[:3] = (action[:3] - self.pose[:3]) / dt
        self.speed[3:] = (
            Rotation.from_rotvec(action[3:6])
            * Rotation.from_rotvec(self.pose[3:]).inv()
        ).as_rotvec() / dt
        self.pose = action[:6].copy()
        self.width = float(action[6])

    def stop(self):
        self.speed[:] = 0

    def close(self):
        self.stop()
