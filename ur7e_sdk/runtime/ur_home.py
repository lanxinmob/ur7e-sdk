import time
from pathlib import Path

import numpy as np
import yaml

from ur7e_sdk.safety import SafetyError, SafetyLayer


def load_homes(config, config_path):
    if config["backend"] != "ur":
        raise ValueError("ARX startup homing requires backend: ur")
    if len(config["arx5"]) != len(config["arms"]):
        raise ValueError("Configure one ARX home reference per UR arm")
    homes = []
    for i, (leader, arm) in enumerate(zip(config["arx5"], config["arms"])):
        settings = leader.get("ur_home", {})
        if not settings.get("reference"):
            raise ValueError(f"Set arx5[{i}].ur_home.reference before teleoperation")
        path = Path(config_path).resolve().parent / settings["reference"]
        with path.open() as stream:
            reference = yaml.safe_load(stream)
        if reference.get("arm_index") != i:
            raise ValueError(f"Reference arm_index must be {i}: {path}")
        q = np.asarray(reference["q_rad"], dtype=float)
        SafetyLayer(arm["safety"]).joint_target(q)
        values = dict(speed=0.1, acceleration=0.1, timeout=120.0)
        if settings.keys() - (values.keys() | {"reference"}):
            raise ValueError("Unknown ur_home setting")
        values.update({k: v for k, v in settings.items() if k != "reference"})
        if any(not np.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError("UR home timing and speed must be finite and positive")
        if values["speed"] > min(0.2, arm["safety"]["max_joint_speed"]):
            raise ValueError("UR home speed must be <= 0.2 rad/s and safety limit")
        if values["acceleration"] > 0.2:
            raise ValueError("UR home acceleration must be <= 0.2 rad/s^2")
        homes.append(dict(q=q, **values))
    return homes


def home_ur(config, homes, poll, factory=None):
    """Own the UR connections until joint homing stops, before the servo env starts."""
    if config["backend"] != "ur":
        raise ValueError("ARX startup homing requires backend: ur")
    if len(homes) != len(config["arms"]):
        raise ValueError("Configure one home target per UR arm")
    if factory is None:
        from ur7e_sdk.hardware.ur import URRobot

        factory = URRobot
    robots, previous = [], []
    guards = [SafetyLayer(a["safety"]) for a in config["arms"]]
    active = None
    last_tick = time.monotonic()

    def check():
        nonlocal last_tick
        poll()
        now = time.monotonic()
        if now - last_tick > min(g.config["max_cycle_lag"] for g in guards):
            raise SafetyError("UR home monitoring missed its deadline")
        last_tick = now
        states = []
        for i, (robot, guard) in enumerate(zip(robots, guards)):
            state = robot.read()
            guard.state(state, previous[i], time.monotonic())
            previous[i] = np.r_[state["pose"], state["width"]]
            if not robot.control.kickWatchdog():
                raise SafetyError("UR home watchdog failed")
            states.append(state)
        return states

    try:
        for arm in config["arms"]:
            poll()
            robots.append(factory(arm, config))
        # Start watchdogs only after all potentially slow connections are open.
        for robot in robots:
            robot.start()
            state = robot.read()
            previous.append(np.r_[state["pose"], state["width"]])
        last_tick = time.monotonic()
        initial = check()
        if any(np.max(np.abs(s["qd"])) > 0.01 for s in initial):
            raise SafetyError("UR must be stationary before joint homing")
        starts = [np.array(s["q"], copy=True) for s in initial]
        # Check all arms before moving any arm. This samples workspace, not collisions.
        for i, (robot, guard, home, start) in enumerate(
            zip(robots, guards, homes, starts)
        ):
            count = max(2, int(np.ceil(np.max(np.abs(home["q"] - start)) / 0.02)) + 1)
            for q in np.linspace(start, home["q"], count):
                check()
                guard.joint_target(q)
                if not robot.control.isJointsWithinSafetyLimits(q.tolist()):
                    raise SafetyError(
                        f"UR{i} home path violates UR joint safety limits"
                    )
                pose = robot.control.getForwardKinematics(
                    q.tolist(), config["arms"][i]["tcp"]
                )
                guard.target(np.r_[pose, previous[i][6]])
            distance = np.max(np.abs(home["q"] - start))
            if (
                distance / home["speed"] + 2 * home["speed"] / home["acceleration"] + 2
                >= home["timeout"]
            ):
                raise ValueError(f"UR{i} home timeout is too short for this move")
        for i, (robot, home, start) in enumerate(zip(robots, homes, starts)):
            state = check()[i]
            if np.max(np.abs(state["q"] - start)) > 0.01:
                raise SafetyError(
                    "UR moved during home planning; restart from current state"
                )
            print(f"UR{i} returning to reference at {home['speed']:.3f} rad/s")
            active = robot
            if not robot.control.moveJ(
                home["q"].tolist(), home["speed"], home["acceleration"], True
            ):
                raise RuntimeError(f"UR{i} rejected joint home command")
            started = time.monotonic()
            settled = None
            while True:
                state = check()[i]
                now = time.monotonic()
                if now - started > home["timeout"]:
                    raise TimeoutError(f"UR{i} did not reach its home joints")
                if np.max(np.abs(state["qd"])) > home["speed"] * 1.5:
                    raise SafetyError(f"UR{i} home measured joint speed exceeded")
                # Native moveJ follows a synchronized straight segment in joint space.
                delta = home["q"] - start
                u = np.clip(
                    np.dot(state["q"] - start, delta)
                    / max(np.dot(delta, delta), 1e-12),
                    0,
                    1,
                )
                if np.max(np.abs(state["q"] - (start + u * delta))) > 0.05:
                    raise SafetyError(
                        f"UR{i} deviated from its checked joint home path"
                    )
                arrived = (
                    np.max(np.abs(state["q"] - home["q"])) <= 0.01
                    and np.max(np.abs(state["qd"])) <= 0.01
                )
                settled = (now if settled is None else settled) if arrived else None
                if settled is not None and now - settled >= 0.2:
                    robot.control.stopJ(0.5)
                    active = None
                    break
                time.sleep(0.02)
        print("UR home complete; establishing teleoperation anchors.")
    finally:
        try:
            if active is not None:
                active.control.stopJ(0.5)
        finally:
            errors = []
            for robot in robots:
                try:
                    robot.close()
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise RuntimeError("UR home cleanup failed: " + "; ".join(errors))
