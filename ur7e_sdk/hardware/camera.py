import threading
import time
from collections import deque

import numpy as np


class Camera:
    """Optional OpenCV RGB stream with host monotonic receive timestamps."""

    def __init__(self, config):
        import cv2

        self.cv2 = cv2
        self.config = config
        self.lock = threading.Lock()
        self.frames = deque(maxlen=config.get("history", 128))
        self.stop_event = threading.Event()
        self.error = None
        self.capture = cv2.VideoCapture(config.get("device", 0))
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError("Cannot open camera")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.get("width", 640))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.get("height", 480))
        self.capture.set(cv2.CAP_PROP_FPS, config.get("fps", 30))
        self.thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                ok, frame = self.capture.read()
                stamp = time.monotonic()
                if not ok:
                    raise RuntimeError("Camera read failed")
                rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
                with self.lock:
                    self.frames.append((stamp, rgb))
        except Exception as exc:
            self.error = exc

    def snapshot(self):
        if self.error:
            raise RuntimeError("Camera failed") from self.error
        with self.lock:
            frames = list(self.frames)
        if not frames:
            raise RuntimeError("Insufficient camera history")
        if time.monotonic() - frames[-1][0] > self.config.get("timeout", 0.5):
            raise RuntimeError("Camera stream is stale")
        return np.array([f[0] for f in frames]), [f[1] for f in frames]

    def close(self):
        self.stop_event.set()
        self.thread.join(1)
        self.capture.release()
        self.thread.join(1)
        if self.thread.is_alive():
            raise RuntimeError("Camera thread did not stop")
