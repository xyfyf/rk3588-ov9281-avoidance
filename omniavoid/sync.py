from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import numpy as np


@dataclass(frozen=True)
class Frame:
    camera: str
    sequence: int
    monotonic_ns: int
    epoch_ns: int
    image: np.ndarray
    timestamp_flags: int = 0


class Synchronizer:
    """Timestamp matching does not create hardware synchronisation.

    Timestamps must already share one verified exposure-time clock domain.
    """
    def __init__(self, names, max_skew_ms, max_age_ms, queue_size=4):
        self.names = tuple(names)
        self.queues = {n: deque(maxlen=queue_size) for n in self.names}
        self.last = {}
        self.max_skew_ns, self.max_age_ns = int(max_skew_ms * 1e6), int(max_age_ms * 1e6)
        self.lock = threading.Lock()
        self.rejected = 0
        self.discarded = 0

    def add(self, frame):
        with self.lock:
            if frame.camera not in self.queues:
                raise ValueError("Unknown camera")
            previous = self.last.get(frame.camera)
            if previous:
                delta = (frame.sequence - previous[0]) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000 or frame.monotonic_ns <= previous[1]:
                    self.rejected += 1
                    return False
            self.last[frame.camera] = (frame.sequence, frame.monotonic_ns)
            if len(self.queues[frame.camera]) == self.queues[frame.camera].maxlen:
                self.discarded += 1
            self.queues[frame.camera].append(frame)
            return True

    def pop(self, now_ns):
        with self.lock:
            latest = None
            for q in self.queues.values():
                while q and now_ns - q[0].monotonic_ns > self.max_age_ns:
                    q.popleft()
                    self.discarded += 1
                if q and q[-1].monotonic_ns > now_ns + self.max_skew_ns:
                    self.discarded += len(q)
                    q.clear()
            while all(self.queues[n] for n in self.names):
                head = {n: self.queues[n][0] for n in self.names}
                oldest = min(head, key=lambda n: head[n].monotonic_ns)
                timestamps = [f.monotonic_ns for f in head.values()]
                if max(timestamps) - min(timestamps) <= self.max_skew_ns:
                    if latest is not None:
                        self.discarded += len(self.names)
                    latest = {n: self.queues[n].popleft() for n in self.names}
                    continue
                self.queues[oldest].popleft()
                self.discarded += 1
            return latest
