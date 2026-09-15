import select
import sys
import termios
import tty

import numpy as np
from scipy.spatial.transform import Rotation


class TerminalKeyboard:
    """Terminal event input: each key produces one bounded increment."""

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "Keyboard teleoperation requires an interactive terminal"
            )
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def read(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            import os

            key = os.read(self.fd, 1).decode("ascii", errors="ignore")
            if key:
                keys.append(key)
        return keys

    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)


class KeyboardTeleop:
    def __init__(self, initial_action, position_step=0.002, rotation_step=0.01):
        self.target = np.array(initial_action, dtype=float, copy=True)
        self.arm = 0
        self.position_step, self.rotation_step = position_step, rotation_step

    def update(self, keys):
        if "q" in keys or "\x1b" in keys:
            raise KeyboardInterrupt
        if "\t" in keys:
            self.arm = (self.arm + 1) % len(self.target)
        translation = dict(
            w=(0, 1), s=(0, -1), a=(1, 1), d=(1, -1), r=(2, 1), f=(2, -1)
        )
        rotation = dict(i=(0, 1), k=(0, -1), j=(1, 1), l=(1, -1), u=(2, 1), o=(2, -1))
        # At most one increment per axis per poll, even with buffered repeats.
        for key in set(keys):
            if key in translation:
                axis, sign = translation[key]
                self.target[self.arm, axis] += sign * self.position_step
            elif key in rotation:
                axis, sign = rotation[key]
                delta = np.zeros(3)
                delta[axis] = sign * self.rotation_step
                self.target[self.arm, 3:6] = (
                    Rotation.from_rotvec(delta)
                    * Rotation.from_rotvec(self.target[self.arm, 3:6])
                ).as_rotvec()
            elif key in ("[", "]"):
                self.target[self.arm, 6] = np.clip(
                    self.target[self.arm, 6] + (0.002 if key == "]" else -0.002),
                    0,
                    0.085,
                )
        return self.target.copy()
