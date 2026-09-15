import time

import numpy as np
from scipy.spatial.transform import Rotation

ALIGNMENT_DEFAULTS = dict(
    max_joint_speed=0.15,
    max_gripper_speed=0.01,
    min_duration=2.0,
    timeout=45.0,
    position_tolerance=0.005,
    rotation_tolerance=0.05,
    gripper_tolerance=0.003,
    max_joint_tracking_error=0.15,
)


def alignment_config(config):
    result = ALIGNMENT_DEFAULTS | config.get("alignment", {})
    if result.keys() != ALIGNMENT_DEFAULTS.keys():
        raise ValueError("Unknown ARX alignment setting")
    if any(not np.isfinite(v) or v <= 0 for v in result.values()):
        raise ValueError("ARX alignment settings must be finite and positive")
    return result


class ARX5Leader:
    """Read local arx5_interface; map relative Cartesian motion to a UR TCP."""

    def __init__(self, config):
        import arx5_interface as arx

        self.config = config
        self.arx = arx
        self.alignment = alignment_config(config)
        axes_rpy = np.asarray(config.get("axes_rpy", [0, 0, 0]), dtype=float)
        tool_rpy = np.asarray(config.get("tool_rpy", [0, 0, 0]), dtype=float)
        if (
            axes_rpy.shape != (3,)
            or tool_rpy.shape != (3,)
            or not np.isfinite(axes_rpy).all()
            or not np.isfinite(tool_rpy).all()
        ):
            raise ValueError(
                "ARX axes_rpy and tool_rpy must contain three finite angles"
            )
        self.axes = Rotation.from_euler("xyz", axes_rpy)
        self.translation = np.asarray(
            config.get("base_translation", [0, 0, 0]), dtype=float
        )
        self.tool_rotation = Rotation.from_euler("xyz", tool_rpy)
        self.scale = float(config.get("position_scale", 1.0))
        if (
            self.translation.shape != (3,)
            or not np.isfinite(self.translation).all()
            or not np.isfinite(self.axes.as_quat()).all()
            or not np.isfinite(self.tool_rotation.as_quat()).all()
            or not np.isfinite(self.scale)
            or self.scale <= 0
        ):
            raise ValueError("Invalid ARX to UR coordinate mapping")
        rc = arx.RobotConfigFactory.get_instance().get_config(config.get("model", "X5"))
        self.robot_config = rc
        self.gripper_max = float(config.get("gripper_width", rc.gripper_width))
        if (
            not np.isfinite(self.gripper_max)
            or not 0 < self.gripper_max <= rc.gripper_width
        ):
            raise ValueError(
                "ARX gripper_width must be within the model's physical range"
            )
        self.solver = arx.Arx5Solver(
            rc.urdf_path,
            rc.joint_dof,
            rc.joint_pos_min,
            rc.joint_pos_max,
            rc.base_link_name,
            rc.eef_link_name,
            rc.gravity_vector,
        )
        cc = arx.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", rc.joint_dof
        )
        cc.gravity_compensation = True
        self.hold_gain = arx.Gain(
            cc.default_kp.copy(),
            cc.default_kd.copy(),
            cc.default_gripper_kp,
            cc.default_gripper_kd,
        )
        self.robot = arx.Arx5JointController(rc, cc, config.get("interface", "can0"))
        gain = self.robot.get_gain()
        self.original_gain = arx.Gain(
            gain.kp().copy(), gain.kd().copy(), gain.gripper_kp, gain.gripper_kd
        )
        self.last_stamp = None
        self.last_update = time.monotonic()
        self.reference = None
        self.reference_width = None
        self.aligned = False

    def enter_teleop(self):
        if not self.aligned:
            raise RuntimeError("ARX alignment has not completed")
        if self.config.get("drag", True):
            gain = self.robot.get_gain()
            gain.kp()[:] = 0
            gain.kd()[:] = self.hold_gain.kd() * 0.05
            gain.gripper_kp = 0
            gain.gripper_kd = 0.05
            self.robot.set_gain(gain)

    def target_from_follower(self, follower):
        follower = np.asarray(follower, dtype=float)
        if (
            follower.shape != (7,)
            or not np.isfinite(follower).all()
            or not 0 <= follower[6] <= 0.085
        ):
            raise ValueError("Invalid UR pose/gripper state for alignment")
        pose = np.empty(6)
        pose[:3] = self.axes.inv().apply(follower[:3] - self.translation) / self.scale
        pose[3:] = (
            self.axes.inv()
            * Rotation.from_rotvec(follower[3:6])
            * self.tool_rotation.inv()
        ).as_euler("xyz")
        return pose, follower[6] / 0.085 * self.gripper_max

    def _joint_state(self):
        state = self.robot.get_joint_state()
        q, qd = np.asarray(state.pos()), np.asarray(state.vel())
        rc = self.robot_config
        if (
            q.shape != (rc.joint_dof,)
            or qd.shape != q.shape
            or not np.isfinite(q).all()
            or not np.isfinite(qd).all()
            or not np.isfinite(state.gripper_pos)
            or np.any(q < rc.joint_pos_min)
            or np.any(q > rc.joint_pos_max)
        ):
            raise RuntimeError("Invalid ARX joint feedback")
        return state

    @staticmethod
    def _pose_error(a, b):
        return (
            np.linalg.norm(a[:3] - b[:3]),
            (
                Rotation.from_euler("xyz", a[3:]).inv()
                * Rotation.from_euler("xyz", b[3:])
            ).magnitude(),
        )

    def plan_alignment(self, follower):
        self.read()
        state = self._joint_state()
        start = np.array(state.pos(), dtype=float, copy=True)
        target, width = self.target_from_follower(follower)
        status, joints = self.solver.inverse_kinematics(target, start)
        joints = np.asarray(joints, dtype=float)
        rc, settings = self.robot_config, self.alignment

        def invalid_solution(status, joints):
            if joints.shape != start.shape or not np.isfinite(joints).all():
                return "IK returned invalid joint values"
            outside = np.flatnonzero(
                (joints < rc.joint_pos_min) | (joints > rc.joint_pos_max)
            )
            if outside.size:
                return "joint limits exceeded: " + "; ".join(
                    f"J{i + 1}={joints[i]:.3f} outside "
                    f"[{rc.joint_pos_min[i]:.3f}, {rc.joint_pos_max[i]:.3f}] rad"
                    for i in outside
                )
            if status < 0:
                return f"IK failed (status {status})"
            return None

        reason = invalid_solution(status, joints)
        if reason:
            # LMA convergence alone does not enforce the physical joint limits.
            retry_status, retry_joints = self.solver.multi_trial_ik(target, start, 20)
            retry_joints = np.asarray(retry_joints, dtype=float)
            retry_reason = invalid_solution(retry_status, retry_joints)
            if retry_reason is None:
                joints = retry_joints
            else:
                raise RuntimeError(
                    f"ARX alignment: {reason}; alternate IK: {retry_reason}. "
                    f"Mapped target (xyz m, xyz RPY rad): {target.tolist()}. "
                    "Calibrate arx5 base_translation/axes_rpy/tool_rpy or change "
                    "the UR starting pose; do not increase physical joint limits."
                )
        actual = np.asarray(self.solver.forward_kinematics(joints))
        if actual.shape != (6,) or not np.isfinite(actual).all():
            raise RuntimeError("ARX alignment FK returned an invalid pose")
        dp, dr = self._pose_error(actual, target)
        if dp > settings["position_tolerance"] or dr > settings["rotation_tolerance"]:
            raise RuntimeError("ARX alignment IK solution does not reach the target")
        joint_speed = min(settings["max_joint_speed"], float(np.min(rc.joint_vel_max)))
        grip_speed = min(settings["max_gripper_speed"], rc.gripper_vel_max)
        if (
            not np.isfinite([joint_speed, grip_speed]).all()
            or min(joint_speed, grip_speed) <= 0
        ):
            raise ValueError("ARX model speed limits must be finite and positive")
        # The zero-velocity/zero-acceleration quintic has peak slope 1.875.
        duration = max(
            settings["min_duration"],
            1.875 * np.max(np.abs(joints - start)) / joint_speed,
            1.875 * abs(width - state.gripper_pos) / grip_speed,
        )
        if duration + 0.2 >= settings["timeout"]:
            raise RuntimeError(
                f"ARX alignment needs {duration:.1f}s, exceeding its timeout"
            )
        return dict(
            pose=target,
            width=width,
            start=start,
            joints=joints,
            start_width=float(state.gripper_pos),
            duration=duration,
            joint_speed=joint_speed,
        )

    def _command(self, joints, width):
        cmd = self.arx.JointState(len(joints))
        cmd.pos()[:] = joints
        cmd.gripper_pos = float(width)
        # Only stream the current small step, never queue the distant endpoint.
        cmd.timestamp = self.robot.get_timestamp()
        self.robot.set_joint_cmd(cmd)

    def hold(self):
        self.read()
        state = self._joint_state()
        self._command(state.pos(), state.gripper_pos)
        self.robot.set_gain(self.hold_gain)

    def align(self, plan, keep_alive, rate=50):
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("Alignment rate must be positive")
        self.aligned = False
        settings = self.alignment
        self.read()
        state = self._joint_state()
        if (
            np.max(np.abs(state.pos() - plan["start"])) > 0.02
            or abs(state.gripper_pos - plan["start_width"]) > 0.003
        ):
            raise RuntimeError(
                "ARX moved after alignment planning; retry from the current pose"
            )
        self._command(plan["start"], plan["start_width"])
        self.robot.set_gain(self.hold_gain)
        started = previous_time = time.monotonic()
        previous_cmd = plan["start"]
        settled_since = None
        try:
            while True:
                keep_alive()
                now = time.monotonic()
                if now - previous_time > self.config.get("state_timeout", 0.25):
                    raise RuntimeError("ARX alignment loop missed its deadline")
                previous_time = now
                pose, width = self.read()
                state = self._joint_state()
                if (
                    np.max(np.abs(state.pos() - previous_cmd))
                    > settings["max_joint_tracking_error"]
                    or np.max(np.abs(state.vel())) > plan["joint_speed"] * 1.5
                ):
                    raise RuntimeError(
                        "ARX alignment joint tracking/speed limit exceeded"
                    )
                elapsed = now - started
                if elapsed > settings["timeout"]:
                    raise TimeoutError("ARX alignment did not settle before timeout")
                u = min(1.0, elapsed / plan["duration"])
                blend = u**3 * (10 - 15 * u + 6 * u**2)
                command = plan["start"] + blend * (plan["joints"] - plan["start"])
                self._command(
                    command,
                    plan["start_width"] + blend * (plan["width"] - plan["start_width"]),
                )
                previous_cmd = command
                dp, dr = self._pose_error(pose, plan["pose"])
                settled = (
                    u == 1
                    and dp <= settings["position_tolerance"]
                    and dr <= settings["rotation_tolerance"]
                    and abs(width - plan["width"]) <= settings["gripper_tolerance"]
                )
                settled_since = (
                    (now if settled_since is None else settled_since)
                    if settled
                    else None
                )
                if settled_since is not None and now - settled_since >= 0.2:
                    self.aligned = True
                    return
                time.sleep(max(0, 1 / rate - (time.monotonic() - now)))
        finally:
            state = self._joint_state()
            self._command(state.pos(), state.gripper_pos)

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
        if not self.aligned:
            raise RuntimeError("Align the ARX leader before establishing anchors")
        self.reference, self.reference_width = self.read()
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
        result[6] = np.clip(
            self.follower[6]
            + (width - self.reference_width) / self.gripper_max * 0.085,
            0,
            0.085,
        )
        return result

    def close(self):
        state = self.robot.get_joint_state()
        cmd = self.arx.JointState(len(state.pos()))
        cmd.pos()[:] = state.pos()
        cmd.gripper_pos = state.gripper_pos
        self.robot.set_joint_cmd(cmd)
        time.sleep(0.02)
        self.robot.set_gain(self.original_gain)
