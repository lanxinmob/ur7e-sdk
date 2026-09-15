import multiprocessing as mp
import queue
import time
import traceback
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from ur7e_sdk.hardware.sim import SimRobot
from ur7e_sdk.safety import SafetyError, SafetyLayer
from ur7e_sdk.shared_memory import SharedMemoryRingBuffer

from .pose_trajectory_interpolator import PoseTrajectoryInterpolator

STATE_WIDTH = 28


def _run(config, commands, history, ready, stop, failed, errors):
    robots = []
    servo_pool = None
    try:
        if config["backend"] == "ur":
            from ur7e_sdk.hardware.ur import URRobot

            factory = URRobot
        else:
            factory = SimRobot
        for arm in config["arms"]:
            robots.append(factory(arm, config))
        for robot in robots:
            robot.start()
        guards = [SafetyLayer(a["safety"]) for a in config["arms"]]
        if len(robots) > 1:
            servo_pool = ThreadPoolExecutor(max_workers=len(robots))
        initial = [r.read() for r in robots]
        now = time.monotonic()
        paths = [PoseTrajectoryInterpolator([now], [s["pose"]]) for s in initial]
        width_times = np.array([now])
        widths = np.array([[s["width"] for s in initial]])
        previous = [np.r_[s["pose"], s["width"]] for s in initial]
        last_command = None
        dt = 1 / config["frequency"]
        tick = now
        while not stop.is_set():
            now = time.monotonic()
            if now - tick > min(g.config["max_cycle_lag"] for g in guards):
                raise SafetyError("Control cycle missed deadline")
            states = [r.read() for r in robots]
            for guard, state, prev in zip(guards, states, previous):
                guard.state(state, prev, time.monotonic())
            try:
                actions, timestamps, submitted = commands.get_nowait()
            except queue.Empty:
                pass
            else:
                if timestamps[0] <= now + dt or timestamps[-1] - now > min(
                    g.config["max_horizon"] for g in guards
                ):
                    raise SafetyError("Action batch expired or exceeds horizon")
                if now - submitted > min(g.config["command_timeout"] for g in guards):
                    raise SafetyError("Stale command producer")
                anchors = [
                    np.r_[p(now), np.interp(now, width_times, widths[:, i])]
                    for i, p in enumerate(paths)
                ]
                # Validate the entire multi-arm batch before replacing any trajectory.
                for i, guard in enumerate(guards):
                    start, start_t = anchors[i], now
                    for action, target_t in zip(actions[:, i], timestamps):
                        guard.segment(start, action, target_t - start_t)
                        start, start_t = action, target_t
                width_times = np.r_[now, timestamps]
                widths = np.vstack([[a[6] for a in anchors], actions[:, :, 6]])
                paths = [
                    PoseTrajectoryInterpolator(
                        width_times, np.vstack([anchors[i][:6], actions[:, i, :6]])
                    )
                    for i in range(len(robots))
                ]
                last_command = submitted
            if last_command is not None and now - last_command > min(
                g.config["command_timeout"] for g in guards
            ):
                raise SafetyError("Command watchdog expired")
            targets = [
                np.r_[p(now), np.interp(now, width_times, widths[:, i])]
                for i, p in enumerate(paths)
            ]
            for guard, target in zip(guards, targets):
                guard.target(target)
            if stop.is_set():
                break
            if servo_pool is None:
                robots[0].servo(targets[0])
            else:
                jobs = [servo_pool.submit(r.servo, a) for r, a in zip(robots, targets)]
                for job in jobs:
                    job.result()
            previous = targets
            row = [now]
            for s in states:
                row.extend(
                    np.r_[
                        s["pose"],
                        s["q"],
                        s["qd"],
                        s["speed"],
                        s["width"],
                        s["timestamp"],
                        s["gripper_timestamp"],
                        float(s["safe"]),
                    ]
                )
            history.put(row)
            ready.set()
            tick += dt
            stop.wait(max(0, tick - time.monotonic()))
    except BaseException:
        failed.set()
        errors.send(traceback.format_exc())
    finally:
        if servo_pool is not None:
            servo_pool.shutdown(wait=True, cancel_futures=True)
        # Stop every arm before closing any potentially slow connection.
        for robot in robots:
            try:
                robot.stop()
            except Exception:
                failed.set()
                errors.send(traceback.format_exc())
        for robot in robots:
            try:
                robot.close()
            except Exception:
                failed.set()
                errors.send(traceback.format_exc())
        errors.close()


class RTDEInterpolationController:
    def __init__(self, config):
        self.config = config
        self.ctx = mp.get_context("spawn")
        self.commands = self.ctx.Queue(maxsize=4)
        self.history = SharedMemoryRingBuffer(
            self.ctx, config["history"], 1 + STATE_WIDTH * len(config["arms"])
        )
        self.ready, self.stop_event, self.failed = (self.ctx.Event() for _ in range(3))
        self.errors, sender = self.ctx.Pipe(duplex=False)
        self.process = self.ctx.Process(
            target=_run,
            name="ur7e-control",
            args=(
                config,
                self.commands,
                self.history,
                self.ready,
                self.stop_event,
                self.failed,
                sender,
            ),
        )
        self._sender = sender
        self._error = None
        self.started = False

    def start(self, timeout=20):
        if self.started:
            raise RuntimeError("Controller cannot be restarted")
        self.started = True
        self.process.start()
        self._sender.close()
        deadline = time.monotonic() + timeout
        try:
            while not self.ready.wait(0.02):
                self.check()
                if time.monotonic() > deadline:
                    raise TimeoutError("Controller startup timed out")
            self.check()
        except BaseException:
            self.stop()
            raise

    def check(self):
        if self.errors.poll():
            try:
                self._error = self.errors.recv()
            except EOFError:
                pass
        if self._error or self.failed.is_set():
            raise SafetyError(self._error or "Controller fault; stop is latched")
        if not self.started or not self.process.is_alive() or self.stop_event.is_set():
            raise RuntimeError("Controller is not running")

    def submit(self, actions, timestamps):
        self.check()
        try:
            self.commands.put_nowait(
                (actions.copy(), timestamps.copy(), time.monotonic())
            )
        except queue.Full as exc:
            self.stop_event.set()
            raise SafetyError("Command queue full; stopping") from exc

    def stop(self):
        self.stop_event.set()
        if self.started and self.process.pid is not None:
            self.process.join(5)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2)
                if self.process.is_alive():
                    self.process.kill()
                    self.process.join()
                self._error = (
                    "Control process required termination; inspect physical stop state"
                )
                self.failed.set()
        self.commands.cancel_join_thread()
        self.commands.close()

    def close(self):
        self.stop()
        if self.failed.is_set():
            if self.errors.poll():
                try:
                    self._error = self.errors.recv()
                except EOFError:
                    pass
            raise SafetyError(self._error or "Controller failed during shutdown")
