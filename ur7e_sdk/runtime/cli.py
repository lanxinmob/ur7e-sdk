import argparse
import importlib
import json
import time
from contextlib import ExitStack

import numpy as np

from ur7e_sdk.env import BimanualUREnv
from ur7e_sdk.policy import load_policy, schedule_output
from ur7e_sdk.runtime.config import load_config
from ur7e_sdk.teleop.keyboard import KeyboardTeleop, TerminalKeyboard


def _warmup(env, steps=1):
    deadline = time.monotonic() + 5
    while True:
        try:
            return env.get_obs(steps)
        except RuntimeError as exc:
            if "history" not in str(exc) or time.monotonic() > deadline:
                raise
            time.sleep(0.02)


def run(args):
    config = load_config(
        args.config,
        require_commissioned=args.command
        not in ("diagnose", "gripper-status", "gripper-activate"),
    )
    if args.command in ("gripper-status", "gripper-activate"):
        from ur7e_sdk.hardware.robotiq import Robotiq2F85

        if config["backend"] != "ur":
            raise ValueError("Gripper commands require backend: ur")
        for arm in config["arms"]:
            device = Robotiq2F85(arm["ip"], arm["gripper"])
            try:
                if args.command == "gripper-activate":
                    device.activate()
                print(
                    json.dumps({k: device.get(k) for k in ("STA", "FLT", "POS", "OBJ")})
                )
            finally:
                device.close()
        return
    if args.command == "observe":
        # The env owns a servo session. Use diagnose for passive real-hardware reads.
        with BimanualUREnv(config) as env:
            obs = _warmup(env)
            print(json.dumps({k: v.tolist() for k, v in obs.items()}, indent=2))
        return
    if args.command == "diagnose":
        if config["backend"] == "sim":
            print("Simulation configuration is valid")
            return
        from rtde_receive import RTDEReceiveInterface

        for arm in config["arms"]:
            r = RTDEReceiveInterface(arm["ip"])
            try:
                print(
                    json.dumps(
                        dict(
                            ip=arm["ip"],
                            pose=r.getActualTCPPose(),
                            joints=r.getActualQ(),
                            safety_mode=r.getSafetyMode(),
                            robot_mode=r.getRobotMode(),
                        ),
                        indent=2,
                    )
                )
            finally:
                r.disconnect()
        return
    policy = load_policy(args.factory) if args.command == "policy" else None
    if args.command == "arx5":
        from ur7e_sdk.teleop.arx5 import ARX5Leader

        try:
            importlib.import_module("arx5_interface")
        except ImportError as exc:
            raise RuntimeError(
                "Cannot import arx5_interface; UR has not been connected. "
                "Set PYTHONPATH to the actual ARX5 SDK python directory "
                "(not /path/to/arx5-sdk) and use the Python version matching "
                f"the compiled binding. Import error: {exc}"
            ) from exc
        leader_configs = config.get("arx5", [])
        if len(leader_configs) != len(config["arms"]):
            raise ValueError("Configure one arx5 entry per UR arm")
        from ur7e_sdk.runtime.ur_home import home_ur, load_homes

        homes = load_homes(config, args.config)
    with ExitStack() as stack:
        leaders = []
        keyboard = None
        if args.command == "arx5":
            keyboard = stack.enter_context(TerminalKeyboard())
            for lc in leader_configs:
                leader = ARX5Leader(lc)
                stack.callback(leader.close)
                leaders.append(leader)
            print("Space: return UR to reference and enable; q/Escape: exit")
            while True:
                keys = keyboard.read()
                if "q" in keys or "\x1b" in keys:
                    return
                for leader in leaders:
                    leader.read()
                if " " in keys:
                    break
                time.sleep(0.02)

            def poll_home():
                if any(key in (" ", "q", "\x1b") for key in keyboard.read()):
                    raise KeyboardInterrupt
                for leader in leaders:
                    leader.read()

            print("UR returning slowly; Space/q/Escape: cancel and exit")
            home_ur(config, homes, poll_home)
        env = stack.enter_context(BimanualUREnv(config))
        obs = _warmup(env, args.obs_steps)
        target = env.action_from_obs(obs)
        if leaders:
            for leader, follower in zip(leaders, target):
                leader.clutch(follower)
            print("Enabled")
        if args.command in ("keyboard", "arx5"):
            if keyboard is None:
                keyboard = stack.enter_context(TerminalKeyboard())
            print("Space: enable/pause; q/Escape: stop; Tab: select arm")
            if args.command == "keyboard":
                print("w/s x, a/d y, r/f z; i/k rx, j/l ry, u/o rz; [/] close/open")
                teleop = KeyboardTeleop(target)
        # Stop the follower before terminal/leader teardown, including exceptions.
        stack.callback(env.controller.stop)
        enabled = args.command in ("policy", "arx5")
        env.exec_action(target[None], [time.time() + args.latency])
        start = time.monotonic()
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            tick = time.monotonic()
            obs = env.get_obs(args.obs_steps)
            if keyboard is not None:
                keys = keyboard.read()
                if "q" in keys or "\x1b" in keys:
                    break
                if " " in keys:
                    enabled = not enabled
                    target = env.action_from_obs(obs)
                    if args.command == "keyboard":
                        teleop.target = target.copy()
                    else:
                        for leader, follower in zip(leaders, target):
                            leader.clutch(follower)
                    print("Enabled" if enabled else "Paused")
                if enabled:
                    target = (
                        teleop.update(keys)
                        if args.command == "keyboard"
                        else np.stack([leader.action() for leader in leaders])
                    )
                elif leaders:
                    for leader in leaders:
                        leader.read()
                actions = target[None]
                timestamps = [time.time() + args.latency]
            else:
                actions, timestamps = schedule_output(
                    policy(obs), obs, args.action_dt, args.latency
                )
            if env.exec_action(actions, timestamps) == 0:
                raise RuntimeError(
                    "All action deadlines expired; inference latency is too high"
                )
            time.sleep(max(0, 1 / args.rate - (time.monotonic() - tick)))


def main():
    parser = argparse.ArgumentParser(description="UR7e SDK")
    parser.add_argument("--config", default="configs/sim.yaml")
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("diagnose", "observe", "gripper-status", "gripper-activate"):
        sub.add_parser(name)
    for name in ("keyboard", "arx5", "policy"):
        p = sub.add_parser(name)
        p.add_argument("--rate", type=float, default=20)
        p.add_argument("--latency", type=float, default=0.2)
        p.add_argument("--duration", type=float, default=0)
        p.add_argument("--obs-steps", type=int, default=1)
        p.add_argument("--action-dt", type=float, default=0.05)
        if name == "policy":
            p.add_argument("--factory", default="ur7e_sdk.policy:HoldPolicy")
    args = parser.parse_args()
    if hasattr(args, "rate"):
        if (
            not np.isfinite(
                [args.rate, args.latency, args.duration, args.action_dt]
            ).all()
            or min(args.rate, args.latency, args.action_dt, args.obs_steps) <= 0
        ):
            parser.error("Invalid loop timing")
    try:
        run(args)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        parser.exit(1, f"{type(exc).__name__}: {exc}\n")


if __name__ == "__main__":
    main()
