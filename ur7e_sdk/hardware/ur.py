import time

import numpy as np

from .robotiq import GripperWorker


class URRobot:
    def __init__(self, arm, config):
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface

        self.control = self.receive = self.gripper = None
        self.config = config
        self.last_robot_time = None
        self.received_at = time.monotonic()
        try:
            self.receive = RTDEReceiveInterface(arm["ip"], config["frequency"])
            self.gripper = GripperWorker(arm["ip"], arm["gripper"])
            self.control = RTDEControlInterface(arm["ip"], config["frequency"])
            if not self.control.setTcp(arm["tcp"]):
                raise RuntimeError("setTcp failed")
            if not self.control.setPayload(arm["payload_mass"], arm["payload_cog"]):
                raise RuntimeError("setPayload failed")
        except BaseException:
            self.close()
            raise

    def start(self):
        if not self.control.setWatchdog(2.0):
            raise RuntimeError("RTDE watchdog setup failed")

    def read(self):
        r = self.receive
        robot_time = r.getTimestamp()
        if self.last_robot_time is None or robot_time > self.last_robot_time:
            self.received_at = time.monotonic()
        self.last_robot_time = robot_time
        width, gt = self.gripper.read()
        return dict(
            pose=np.asarray(r.getActualTCPPose()),
            q=np.asarray(r.getActualQ()),
            qd=np.asarray(r.getActualQd()),
            speed=np.asarray(r.getActualTCPSpeed()),
            width=width,
            timestamp=self.received_at,
            gripper_timestamp=gt,
            safe=(
                r.isConnected()
                and self.control.isConnected()
                and r.getSafetyMode() in (1, 2)
                and r.getRobotMode() == 7
            ),
        )

    def servo(self, action):
        c = self.config
        if not self.control.isPoseWithinSafetyLimits(action[:6].tolist()):
            raise RuntimeError("UR pose/IK safety check failed")
        if not self.control.kickWatchdog():
            raise RuntimeError("UR watchdog kick failed")
        if not self.control.servoL(
            action[:6].tolist(),
            0.5,
            0.5,
            1 / c["frequency"],
            c["lookahead_time"],
            c["gain"],
        ):
            raise RuntimeError("UR servoL failed")
        self.gripper.command(action[6])

    def stop(self):
        if self.gripper is not None:
            self.gripper.stop_event.set()
        if self.control is not None:
            if not self.control.servoStop():
                raise RuntimeError("UR servoStop failed")

    def close(self):
        errors = []
        if self.control is not None:
            for method in (
                self.control.servoStop,
                self.control.stopScript,
                self.control.disconnect,
            ):
                try:
                    method()
                except Exception as exc:
                    errors.append(str(exc))
            self.control = None
        if self.gripper is not None:
            try:
                self.gripper.close()
            except Exception as exc:
                errors.append(str(exc))
            self.gripper = None
        if self.receive is not None:
            try:
                self.receive.disconnect()
            except Exception as exc:
                errors.append(str(exc))
            self.receive = None
        if errors:
            raise RuntimeError("Hardware cleanup failed: " + "; ".join(errors))
