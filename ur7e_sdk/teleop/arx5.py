import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class ARX5Leader:
    """Read local arx5_interface; map relative Cartesian motion to a UR TCP."""

    def __init__(self, config):
        import arx5_interface as arx

        self.config = config
        self.arx = arx
        rc = arx.RobotConfigFactory.get_instance().get_config(config.get("model", "X5"))
        if config.get("urdf_path"):
            path = Path(config["urdf_path"]).resolve()
            model_name = ET.parse(path).getroot().get("name")
            if model_name != config.get("model", "X5"):
                raise ValueError(
                    f"ARX URDF model {model_name!r} does not match configured model"
                )
            rc.urdf_path = str(path)
        cc = arx.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", rc.joint_dof
        )
        cc.gravity_compensation = True
        cc.background_send_recv = True
        gravity_scale = float(config.get("gravity_scale", 1.0))
        kd_scale = float(config.get("drag_kd_scale", 0.05))
        gripper_kd = float(config.get("drag_gripper_kd", 0.05))
        gravity = np.asarray(
            config.get("gravity_vector", rc.gravity_vector), dtype=float
        )
        if (
            not np.isfinite([gravity_scale, kd_scale, gripper_kd]).all()
            or not 0 < gravity_scale <= 2
            or not 0 <= kd_scale <= 1
            or gripper_kd < 0
            or gravity.shape != (3,)
            or not np.isfinite(gravity).all()
            or not 9 <= np.linalg.norm(gravity) <= 10
        ):
            raise ValueError("Invalid ARX drag gravity/damping settings")
        # Robot configs are copied by the native constructor; set these beforehand.
        rc.gravity_vector = gravity * gravity_scale
        open_readout = float(config.get("gripper_open_readout", -3.4))
        if not np.isfinite(open_readout):
            raise ValueError("Invalid ARX gripper_open_readout")
        if open_readout != 0:
            rc.gripper_open_readout = open_readout
        self.last_stamp = None
        self.last_update = time.monotonic()
        self.reference = None
        self.axes = Rotation.from_euler("xyz", config.get("axes_rpy", [0, 0, 0]))
        self.scale = float(config.get("position_scale", 1.0))
        self.gripper_max = float(config.get("gripper_width", rc.gripper_width))
        if not np.isfinite(self.scale) or self.scale <= 0 or not self.gripper_max > 0:
            raise ValueError("Invalid ARX position scale/gripper width")
        self.robot = arx.Arx5JointController(rc, cc, config.get("interface", "can0"))
        gain = self.robot.get_gain()
        self.original_gain = arx.Gain(
            gain.kp().copy(), gain.kd().copy(), gain.gripper_kp, gain.gripper_kd
        )
        if config.get("drag", True):
            try:
                # Clear any startup velocity/torque in the interpolator without homing.
                state = self.robot.get_joint_state()
                cmd = arx.JointState(rc.joint_dof)
                cmd.pos()[:] = state.pos()
                cmd.vel()[:] = 0
                cmd.torque()[:] = 0
                cmd.gripper_pos = state.gripper_pos
                cmd.gripper_vel = 0
                cmd.gripper_torque = 0
                cmd.timestamp = self.robot.get_timestamp()
                self.robot.set_joint_cmd(cmd)
                gain = arx.Gain(
                    np.zeros(rc.joint_dof), cc.default_kd * kd_scale, 0, gripper_kd
                )
                self.robot.set_gain(gain)
                actual = self.robot.get_gain()
                actual_cc = self.robot.get_controller_config()
                actual_rc = self.robot.get_robot_config()
                if (
                    not actual_cc.gravity_compensation
                    or not actual_cc.background_send_recv
                    or not np.allclose(actual.kp(), 0)
                    or not np.allclose(actual.kd(), gain.kd())
                    or not np.isclose(actual.gripper_kp, 0)
                    or not np.isclose(actual.gripper_kd, gripper_kd)
                    or not np.allclose(actual_rc.gravity_vector, rc.gravity_vector)
                    or actual_rc.urdf_path != rc.urdf_path
                ):
                    raise RuntimeError("ARX drag settings did not take effect")
                print(
                    f"ARX drag: {config.get('model', 'X5')} {config.get('interface', 'can0')}, "
                    f"gravity={np.round(actual_rc.gravity_vector, 4).tolist()}, "
                    f"kp=0, kd={np.round(actual.kd(), 4).tolist()}, "
                    f"gripper_open_readout={actual_rc.gripper_open_readout:.4f}"
                )
                print(f"ARX binding: {getattr(arx, '__file__', 'unknown')}")
                print(f"ARX URDF: {actual_rc.urdf_path}")
            except BaseException:
                self.robot.set_to_damping()
                raise

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
