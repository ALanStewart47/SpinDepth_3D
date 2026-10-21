"""Collect a fixed per-region ground reference from complete frames."""

import numpy as np


class GroundSampler:
    def __init__(self):
        self._distances = []
        self._valid = []
        self._last_frame = None

    @property
    def count(self):
        return len(self._distances)

    @property
    def complete(self):
        return self.count == 30

    def add(self, frame):
        if self.complete or frame is self._last_frame:
            return False
        self._last_frame = frame
        self._distances.append(frame.distance_mm.astype(float).copy())
        self._valid.append(frame.valid.copy())
        return True

    def result(self):
        if not self.complete:
            return None
        distances = np.stack(self._distances)
        valid = np.stack(self._valid)
        counts = valid.sum(axis=0)
        ground = np.full((8, 8), np.nan)
        for row, col in zip(*np.where(counts >= 24)):
            ground[row, col] = np.median(distances[valid[:, row, col], row, col])
        return ground if np.isfinite(ground).any() else None
