import time

import numpy as np
from scipy.spatial.transform import Rotation


class ARX5Leader:
    """Read local arx5_interface; map relative Cartesian motion to a UR TCP."""

    def __init__(self, config):
        import arx5_interface as arx

        self.config = config
        self.arx = arx
        rc = arx.RobotConfigFactory.get_instance().get_config(config.get("model", "X5"))
        cc = arx.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", rc.joint_dof
        )
        cc.gravity_compensation = True
        self.robot = arx.Arx5JointController(rc, cc, config.get("interface", "can0"))
        gain = self.robot.get_gain()
        self.original_gain = arx.Gain(
            gain.kp().copy(), gain.kd().copy(), gain.gripper_kp, gain.gripper_kd
        )
        self.last_stamp = None
        self.last_update = time.monotonic()
        self.reference = None
        self.axes = Rotation.from_euler("xyz", config.get("axes_rpy", [0, 0, 0]))
        self.scale = float(config.get("position_scale", 1.0))
        self.gripper_max = float(config.get("gripper_width", rc.gripper_width))
        if not np.isfinite(self.scale) or self.scale <= 0 or not self.gripper_max > 0:
            raise ValueError("Invalid ARX position scale/gripper width")
        if config.get("drag", True):
            gain = self.robot.get_gain()
            gain.kp()[:] = 0
            gain.kd()[:] = cc.default_kd * 0.05
            gain.gripper_kp = 0
            gain.gripper_kd = 0.05
            self.robot.set_gain(gain)

    def read(self):
        if self.robot.is_emergency():
            raise RuntimeError("ARX leader is in emergency state")
        state = self.robot.get_eef_state()
        pose = np.array(state.pose_6d(), dtype=float, copy=True)
        if (
            pose.shape != (6,)
            or not np.isfinite(pose).all()
            or not np.isfinite(state.gripper_pos)
            or not np.isfinite(state.timestamp)
        ):
            raise RuntimeError("Invalid ARX feedback")
        if self.last_stamp is not None and state.timestamp < self.last_stamp:
            raise RuntimeError("ARX timestamp moved backwards")
        if self.last_stamp is None or state.timestamp > self.last_stamp:
            self.last_update = time.monotonic()
        self.last_stamp = state.timestamp
        if time.monotonic() - self.last_update > self.config.get("state_timeout", 0.25):
            raise RuntimeError("ARX feedback stopped updating")
        return pose, float(state.gripper_pos)

    def clutch(self, follower):
        self.reference = self.read()[0]
        self.follower = np.array(follower, copy=True)

    def action(self):
        pose, width = self.read()
        if self.reference is None:
            raise RuntimeError("Clutch the leader before enabling motion")
        result = self.follower.copy()
        result[:3] += self.scale * self.axes.apply(pose[:3] - self.reference[:3])
        delta = (
            Rotation.from_euler("xyz", pose[3:])
            * Rotation.from_euler("xyz", self.reference[3:]).inv()
        )
        result[3:6] = (
            self.axes
            * delta
            * self.axes.inv()
            * Rotation.from_rotvec(self.follower[3:6])
        ).as_rotvec()
        result[6] = np.clip(width / self.gripper_max, 0, 1) * 0.085
        return result

    def close(self):
        state = self.robot.get_joint_state()
        cmd = self.arx.JointState(len(state.pos()))
        cmd.pos()[:] = state.pos()
        cmd.gripper_pos = state.gripper_pos
        self.robot.set_joint_cmd(cmd)
        time.sleep(0.02)
        self.robot.set_gain(self.original_gain)
