"""Decode the existing STM32 demo's ASCII stream without changing its firmware."""

from dataclasses import dataclass
import logging
import re
import time

import numpy as np


MAX_DISTANCE_MM = 2000


@dataclass
class Frame:
    sequence: int
    target_sum: int
    received_at: float  # Host monotonic time, not the sensor's acquisition time.
    distance_mm: np.ndarray
    status: np.ndarray
    targets: np.ndarray

    @property
    def valid(self):
        return ((self.targets > 0) & np.isin(self.status, (5, 9))
                & (self.distance_mm >= 0) & (self.distance_mm <= MAX_DISTANCE_MM))


class FrameParser:
    def __init__(self):
        self.errors = 0
        self._line = bytearray()
        self._skip_line = False
        self._header = None
        self._rows = {}

    def _reject(self):
        self.errors += 1
        self._header = None
        self._rows.clear()

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for byte in data:
            if byte == 10:
                if not self._skip_line:
                    frame = self._parse_line(bytes(self._line))
                    if frame is not None:
                        frames.append(frame)
                self._line.clear()
                self._skip_line = False
            elif not self._skip_line:
                self._line.append(byte)
                if len(self._line) > 256:
                    self._reject()
                    self._line.clear()
                    self._skip_line = True
        return frames

    def _parse_line(self, line):
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError:
            self._reject()
            return None
        header = re.fullmatch(r"FRAME (\d{1,3}) target_sum=(\d{1,5})", text)
        if header:
            if self._header is not None:
                self._reject()
            sequence, total = map(int, header.groups())
            if sequence > 255 or total > 64 * 255:
                self._reject()
            else:
                self._header = (sequence, total)
            return None
        if text.startswith("ERROR "):
            logging.warning("Device: %s", text)
        if not text or text in ("D(mm)", "STATUS (5/9 valid)", "TARGETS"):
            return None
        if self._header is None:
            return None
        row = re.fullmatch(r"([DSN])([0-7]):\s*(.*)", text)
        if row is None:
            self._reject()
            return None
        kind, index, body = row.groups()
        key = (kind, int(index))
        try:
            values = [int(value) for value in body.split()]
        except ValueError:
            self._reject()
            return None
        low, high = (-32768, 32767) if kind == "D" else (0, 255)
        if key in self._rows or len(values) != 8 or any(not low <= x <= high for x in values):
            self._reject()
            return None
        self._rows[key] = values
        if len(self._rows) != 24:
            return None
        distance = np.array([self._rows["D", i] for i in range(8)], dtype=np.int16)
        status = np.array([self._rows["S", i] for i in range(8)], dtype=np.uint8)
        targets = np.array([self._rows["N", i] for i in range(8)], dtype=np.uint8)
        sequence, total = self._header
        if int(targets.sum()) != total:
            self._reject()
            return None
        self._header = None
        self._rows.clear()
        return Frame(sequence, total, time.monotonic(), distance, status, targets)
