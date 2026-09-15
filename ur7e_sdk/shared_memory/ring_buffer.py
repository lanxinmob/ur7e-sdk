from contextlib import contextmanager

import numpy as np


class SharedMemoryRingBuffer:
    """Fixed-size process-shared numeric history with atomic copied snapshots."""

    def __init__(self, ctx, capacity, width):
        self.capacity, self.width = capacity, width
        self._data = ctx.RawArray("d", capacity * width)
        self._count = ctx.Value("Q", 0, lock=False)
        self._lock = ctx.Lock()

    @contextmanager
    def _locked(self):
        if not self._lock.acquire(timeout=0.2):
            raise RuntimeError("Shared history lock timed out")
        try:
            yield
        finally:
            self._lock.release()

    def put(self, row):
        row = np.asarray(row, dtype=float)
        if row.shape != (self.width,):
            raise ValueError("Invalid ring row")
        with self._locked():
            view = np.frombuffer(self._data).reshape(self.capacity, self.width)
            view[self._count.value % self.capacity] = row
            self._count.value += 1

    def get_all(self):
        with self._locked():
            end = self._count.value
            indices = np.arange(max(0, end - self.capacity), end) % self.capacity
            return (
                np.frombuffer(self._data)
                .reshape(self.capacity, self.width)[indices]
                .copy()
            )
