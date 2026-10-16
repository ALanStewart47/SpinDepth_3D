"""Synthetic wire data for tests only; not a hardware capture."""

import numpy as np


def wire_frame(sequence=1, distance=None, status=None, targets=None):
    distance = np.full((8, 8), 300) if distance is None else distance
    status = np.full((8, 8), 5) if status is None else status
    targets = np.ones((8, 8), dtype=int) if targets is None else targets
    text = f"\r\nFRAME {sequence} target_sum={int(targets.sum())}\r\nD(mm)\r\n"
    for kind, array, width in (("D", distance, 5), ("S", status, 2), ("N", targets, 2)):
        if kind == "S":
            text += "STATUS (5/9 valid)\r\n"
        if kind == "N":
            text += "TARGETS\r\n"
        for row in range(8):
            text += f"{kind}{row}:" + "".join(f" {int(v):{width}d}" for v in array[row]) + "\r\n"
    return text.encode("ascii")
