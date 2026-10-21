import time
import unittest
from unittest.mock import patch

import numpy as np
import serial

from main import display_data, preview_data, surface_triangles
from protocol import FrameParser
from serial_link import SerialReader
from samples import wire_frame


class ParserTests(unittest.TestCase):
    def test_startup_chunks_and_concatenated_frames(self):
        startup = b"VL53L8CH demo: 8x8, 5 Hz, distance in mm\r\nInit OK, module type L8\r\n"
        stream = startup + wire_frame(254) + wire_frame(0) + wire_frame(1)
        for size in (1, 7, 96, 4096):
            with self.subTest(size=size):
                parser = FrameParser()
                frames = []
                for offset in range(0, len(stream), size):
                    frames.extend(parser.feed(stream[offset:offset + size]))
                self.assertEqual([frame.sequence for frame in frames], [254, 0, 1])
                self.assertEqual(parser.errors, 0)
                self.assertEqual(frames[-1].distance_mm[0, 0], 300)

    def test_partial_frame_never_published(self):
        parser = FrameParser()
        data = wire_frame()
        self.assertEqual(parser.feed(data[:-1]), [])
        self.assertEqual(len(parser.feed(data[-1:])), 1)

    def test_mid_frame_connection(self):
        parser = FrameParser()
        data = wire_frame()
        frames = parser.feed(data[150:] + wire_frame(2))
        self.assertEqual([frame.sequence for frame in frames], [2])

    def test_bad_frames_discarded_and_next_frame_recovers(self):
        data = wire_frame()
        first_row = data[data.index(b"D0:"):data.index(b"D1:")]
        cases = {
            "missing row": data.replace(first_row, b""),
            "duplicate row": data.replace(first_row, first_row * 2),
            "short row": data.replace(first_row, b"D0: 100 200\r\n"),
            "invalid number": data.replace(first_row, b"D0: x 1 2 3 4 5 6 7\r\n"),
            "distance overflow": data.replace(first_row, b"D0: 99999 1 2 3 4 5 6 7\r\n"),
            "negative status": data.replace(b"S0:", b"S0: -1"),
            "wrong total": data.replace(b"target_sum=64", b"target_sum=63"),
            "invalid byte": data.replace(b"D0:", b"D0:\xff"),
            "long line": data.replace(first_row, b"x" * 10000 + b"\r\n"),
            "unknown row": data.replace(b"D0:", b"D8:"),
            "restart": data[:150] + b"\r\nVL53L8CH demo: restarting\r\n",
        }
        for name, damaged in cases.items():
            with self.subTest(name=name):
                parser = FrameParser()
                frames = parser.feed(damaged + wire_frame(7))
                self.assertEqual([frame.sequence for frame in frames], [7])
                self.assertGreater(parser.errors, 0)

    def test_out_of_range_header(self):
        for bad in (b"FRAME 999 target_sum=64\r\n", b"FRAME 1 target_sum=99999\r\n"):
            parser = FrameParser()
            self.assertEqual(parser.feed(bad), [])
            self.assertEqual(len(parser.feed(wire_frame())), 1)

    def test_line_memory_bounded_and_does_not_parse_suffix(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(b"x" * 100000), [])
        self.assertLessEqual(len(parser._line), 256)
        self.assertEqual(parser.feed(b"FRAME 1 target_sum=64\r\n"), [])
        self.assertEqual(len(parser.feed(wire_frame())), 1)


class PreviewTests(unittest.TestCase):
    def test_validity_and_index_geometry(self):
        distance = np.arange(64).reshape(8, 8) + 200
        status = np.full((8, 8), 5)
        targets = np.ones((8, 8), dtype=int)
        status[0, 1], status[0, 2], targets[0, 3] = 9, 255, 0
        frame = FrameParser().feed(wire_frame(distance=distance, status=status, targets=targets))[0]
        points, colors, grid = preview_data(frame)
        self.assertEqual(frame.valid.sum(), 62)
        self.assertTrue(frame.valid[0, 1])
        self.assertFalse(frame.valid[0, 2])
        self.assertFalse(frame.valid[0, 3])
        np.testing.assert_allclose(points[0], [-3.5, 3.5, -2])
        np.testing.assert_allclose(points[1], [-2.5, 3.5, -2.01])
        np.testing.assert_allclose(points[-1], [3.5, -3.5, -2.63])
        self.assertEqual(colors.shape, (62, 3))
        np.testing.assert_allclose(grid[0, 2], [0.22, 0.24, 0.27])

    def test_all_invalid_has_no_points(self):
        frame = FrameParser().feed(wire_frame(status=np.full((8, 8), 255), targets=np.zeros((8, 8))))[0]
        points, colors, _ = preview_data(frame)
        self.assertEqual(points.shape, (0, 3))
        self.assertEqual(colors.shape, (0, 3))

    def test_outside_test_range_has_no_points_but_preserves_raw_distance(self):
        distance = np.full((8, 8), 2000)
        frame = FrameParser().feed(wire_frame(distance=distance))[0]
        points, colors, grid = preview_data(frame)
        self.assertEqual(points.shape, (0, 3))
        self.assertEqual(colors.shape, (0, 3))
        np.testing.assert_allclose(grid, np.broadcast_to([0.22, 0.24, 0.27], grid.shape))
        np.testing.assert_array_equal(frame.distance_mm, distance)

    def test_one_metre_boundary_and_negative_distance(self):
        distance = np.full((8, 8), 500)
        distance[0] = [-1, 0, 1, 999, 1000, 1001, 1000, 500]
        status = np.full((8, 8), 5)
        status[0, 4], status[0, 6] = 9, 255
        targets = np.ones((8, 8), dtype=int)
        targets[0, 7] = 0
        frame = FrameParser().feed(wire_frame(distance=distance, status=status, targets=targets))[0]
        np.testing.assert_array_equal(frame.valid[0], [False, True, True, True, True, False, False, False])
        points, colors, _ = preview_data(frame)
        self.assertEqual(len(points), 60)
        self.assertEqual(points[3, 2], -10)
        np.testing.assert_allclose(colors[3], [0.12, 0.38, 0.68])


class SurfaceTests(unittest.TestCase):
    def test_full_grid_connects_neighbors_with_upward_faces(self):
        triangles = surface_triangles(np.ones((8, 8), dtype=bool))
        self.assertEqual(triangles.shape, (98, 3))
        frame = FrameParser().feed(wire_frame())[0]
        points, _, _ = preview_data(frame)
        faces = points[triangles]
        normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
        self.assertTrue((normals[:, 2] > 0).all())

    def test_missing_row_is_not_bridged_and_indices_match_compacted_points(self):
        valid = np.ones((8, 8), dtype=bool)
        valid[3, :] = False
        triangles = surface_triangles(valid)
        self.assertEqual(len(triangles), 70)
        rows, cols = np.where(valid)
        self.assertLess(triangles.max(), valid.sum())
        self.assertGreaterEqual(triangles.min(), 0)
        self.assertTrue((np.ptp(rows[triangles], axis=1) == 1).all())
        self.assertTrue((np.ptp(cols[triangles], axis=1) == 1).all())

    def test_empty_and_isolated_points_have_no_faces(self):
        for valid in (np.zeros((8, 8), dtype=bool), np.eye(8, dtype=bool)):
            self.assertEqual(surface_triangles(valid).shape, (0, 3))

    def test_relative_surface_uses_current_and_reference_validity(self):
        targets = np.ones((8, 8), dtype=int)
        targets[0, 0] = 0
        frame = FrameParser().feed(wire_frame(targets=targets))[0]
        reference = np.full((8, 8), 400.0)
        reference[7, 7] = np.nan
        _, valid = display_data(frame, reference)
        points, _, _ = preview_data(frame, reference)
        faces = points[surface_triangles(valid)]
        self.assertTrue(np.isfinite(faces).all())
        self.assertEqual(len(points), 62)
        self.assertFalse(np.any(np.all(faces == [-3.5, 3.5, 1], axis=2)))
        self.assertFalse(np.any(np.all(faces == [3.5, -3.5, 1], axis=2)))


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.closed = False

    def read(self, size):
        if self.chunks:
            item = self.chunks.pop(0)
            if isinstance(item, Exception):
                raise item
            return item
        time.sleep(0.01)
        return b""

    def close(self):
        self.closed = True


class ReceiverTests(unittest.TestCase):
    def wait_for(self, reader, predicate):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            state = reader.snapshot()
            if predicate(state):
                return state
            time.sleep(0.01)
        self.fail("receiver did not reach expected state")

    def test_latest_frame_only_and_serial_closed(self):
        device = FakeSerial([wire_frame(1), wire_frame(2), wire_frame(3)])
        with patch("serial_link.serial.Serial", return_value=device) as connect:
            reader = SerialReader("COM_TEST")
        try:
            state = self.wait_for(reader, lambda state: state.frames == 3)
            self.assertEqual(state.frame.sequence, 3)
            connect.assert_called_once_with("COM_TEST", baudrate=115200, timeout=0.1)
        finally:
            reader.close()
        self.assertTrue(device.closed)
        self.assertFalse(reader._thread.is_alive())

    def test_disconnect_releases_serial_and_preserves_error(self):
        device = FakeSerial([wire_frame(), serial.SerialException("unplugged")])
        with patch("serial_link.serial.Serial", return_value=device):
            reader = SerialReader("COM_TEST")
        try:
            state = self.wait_for(reader, lambda state: bool(state.error))
            self.assertEqual(state.error, "unplugged")
            self.assertEqual(state.frames, 1)
        finally:
            reader.close()
        self.assertTrue(device.closed)


if __name__ == "__main__":
    unittest.main()
