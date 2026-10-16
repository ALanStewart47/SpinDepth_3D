"""One receiver thread and one latest-frame slot; no growing display queue."""

from collections import deque
from dataclasses import dataclass
import threading
import time

import serial

from protocol import Frame, FrameParser


@dataclass
class ReceiverState:
    frame: Frame | None
    frames: int
    errors: int
    fps: float
    error: str


class SerialReader:
    def __init__(self, port):
        self._serial = serial.Serial(port, baudrate=115200, timeout=0.1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame = None
        self._frames = 0
        self._errors = 0
        self._error = ""
        self._times = deque(maxlen=25)
        self._thread = threading.Thread(target=self._receive, name="tof-serial", daemon=True)
        self._thread.start()

    def _receive(self):
        parser = FrameParser()
        try:
            while not self._stop.is_set():
                frames = parser.feed(self._serial.read(4096))
                with self._lock:
                    self._errors = parser.errors
                    if frames:
                        self._frame = frames[-1]
                        self._frames += len(frames)
                        self._times.extend(frame.received_at for frame in frames)
        except (serial.SerialException, OSError) as exc:
            with self._lock:
                self._error = str(exc)
        finally:
            self._serial.close()

    def snapshot(self):
        with self._lock:
            fps = 0.0
            if len(self._times) > 1 and time.monotonic() - self._times[-1] < 2:
                elapsed = self._times[-1] - self._times[0]
                if elapsed > 0:
                    fps = (len(self._times) - 1) / elapsed
            return ReceiverState(self._frame, self._frames, self._errors, fps, self._error)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
