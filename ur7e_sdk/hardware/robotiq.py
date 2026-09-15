import re
import select
import socket
import threading
import time


class Robotiq2F85:
    """Robotiq URCap ASCII socket service; widths are calibrated estimates in m."""

    def __init__(self, host, config):
        self.config = config
        self.socket = socket.create_connection(
            (host, config["port"]), config["timeout"]
        )
        self.socket.settimeout(config["timeout"])
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _request(self, command, variable=None):
        self.socket.sendall((command + "\n").encode("ascii"))
        data = b""
        deadline = time.monotonic() + self.config["timeout"]
        # Some URCap versions omit a line terminator. Allow a bounded idle gap
        # only after a complete reply; keep reading fragmented numeric replies.
        while time.monotonic() < deadline:
            complete = (
                data.strip() == b"ack"
                if variable is None
                else re.fullmatch(rb"[A-Z]+ -?\d+\s*", data) is not None
            )
            wait = min(
                0.01 if complete else self.config["timeout"],
                max(0, deadline - time.monotonic()),
            )
            if not select.select([self.socket], [], [], wait)[0]:
                if complete:
                    break
                continue
            chunk = self.socket.recv(128)
            if not chunk:
                raise ConnectionError("Robotiq socket closed")
            data += chunk
            if len(data) > 256:
                raise RuntimeError("Oversized Robotiq reply")
            if b"\n" in data or (variable is None and data.strip() == b"ack"):
                break
        reply = data.decode("ascii").strip()
        if variable is None:
            if reply != "ack":
                raise RuntimeError(f"Robotiq rejected command: {reply!r}")
            return None
        fields = reply.split()
        if len(fields) != 2 or fields[0] != variable:
            raise RuntimeError(f"Malformed Robotiq reply: {reply!r}")
        return int(fields[1])

    def get(self, variable):
        return self._request(f"GET {variable}", variable)

    def set(self, **values):
        self._request("SET " + " ".join(f"{k} {int(v)}" for k, v in values.items()))

    def activate(self, timeout=10):
        if self.get("STA") == 3:
            return
        self.set(ACT=0, ATR=0)
        deadline = time.monotonic() + timeout
        while self.get("STA") != 0 or self.get("ACT") != 0:
            if time.monotonic() > deadline:
                raise TimeoutError("Robotiq reset timed out")
            time.sleep(0.05)
        self.set(ACT=1)
        while self.get("STA") != 3:
            if time.monotonic() > deadline:
                raise TimeoutError("Robotiq activation timed out")
            time.sleep(0.05)

    def width(self):
        raw = self.get("POS")
        if not 0 <= raw <= 255:
            raise RuntimeError("Invalid Robotiq position")
        g = self.config
        return max(
            0.0,
            min(
                0.085,
                0.085 * (g["closed_raw"] - raw) / (g["closed_raw"] - g["open_raw"]),
            ),
        )

    def move(self, width):
        if not 0 <= width <= 0.085:
            raise ValueError("Invalid gripper width")
        g = self.config
        raw = round(g["closed_raw"] - width / 0.085 * (g["closed_raw"] - g["open_raw"]))
        self.set(POS=raw, SPE=g["speed"], FOR=g["force"], GTO=1)

    def close(self):
        self.socket.close()


class GripperWorker:
    def __init__(self, host, config):
        self.device = Robotiq2F85(host, config)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.target = None
        try:
            if self.device.get("STA") != 3 or self.device.get("FLT") != 0:
                raise RuntimeError(
                    "Robotiq must be activated and fault-free before control"
                )
            self.value = self.device.width()
            self.timestamp = time.monotonic()
        except BaseException:
            self.device.close()
            raise
        self.thread = threading.Thread(target=self._run, name="robotiq", daemon=True)
        self.thread.start()

    def _run(self):
        previous = None
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    target = self.target
                if target is not None and (
                    previous is None or abs(target - previous) >= 0.0003
                ):
                    self.device.move(target)
                    previous = target
                if self.device.get("FLT") != 0 or self.device.get("STA") != 3:
                    raise RuntimeError("Robotiq fault or lost activation")
                width = self.device.width()
                with self.lock:
                    self.value, self.timestamp = width, time.monotonic()
                self.stop_event.wait(0.02)
        except Exception as exc:
            self.error = exc
        finally:
            try:
                self.device.set(GTO=0)
            except Exception as exc:
                self.error = self.error or exc
            self.device.close()

    def read(self):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        with self.lock:
            return self.value, self.timestamp

    def command(self, width):
        with self.lock:
            self.target = float(width)

    def close(self):
        self.stop_event.set()
        self.thread.join(2)
        if self.thread.is_alive():
            self.device.close()
            raise RuntimeError("Gripper worker did not stop")
