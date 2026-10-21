import unittest

import numpy as np

from ground_reference import GroundSampler
from main import display_data, preview_data
from protocol import Frame


def frame(sequence, distance, valid=None):
    valid = np.ones((8, 8), dtype=bool) if valid is None else valid
    status = np.full((8, 8), 5, dtype=np.uint8)
    status[1::2] = 9
    status[~valid] = 255
    return Frame(sequence, int(valid.sum()), float(sequence), np.asarray(distance, dtype=np.int16),
                 status, valid.astype(np.uint8))


class GroundReferenceTests(unittest.TestCase):
    def test_per_region_median_and_relative_rise(self):
        ground = np.arange(64, dtype=float).reshape(8, 8) + 600
        sampler = GroundSampler()
        for sequence in range(30):
            sampler.add(frame(sequence, ground + (sequence % 3 - 1)))
        reference = sampler.result()
        np.testing.assert_allclose(reference, ground)
        current = frame(31, ground.copy())
        current.distance_mm[2, 3] -= 100
        values, valid = display_data(current, reference)
        self.assertTrue(valid.all())
        self.assertEqual(values[2, 3], 100)
        self.assertEqual(np.count_nonzero(values), 1)
        points, _, _ = preview_data(current, reference)
        self.assertEqual(points.shape, (64, 3))
        self.assertEqual(points[2 * 8 + 3, 2], 1)

    def test_distinct_complete_frames_and_partial_reference(self):
        sampler = GroundSampler()
        valid = np.ones((8, 8), dtype=bool)
        valid[0, 0] = False
        first = frame(0, np.full((8, 8), 700), valid)
        sampler.add(first)
        self.assertFalse(sampler.add(first))
        for sequence in range(1, 30):
            current_valid = valid.copy()
            if sequence > 6:
                current_valid[0, 0] = True
            sampler.add(frame(sequence, np.full((8, 8), 700 + sequence), current_valid))
        reference = sampler.result()
        self.assertTrue(np.isnan(reference[0, 0]))
        self.assertEqual(reference[0, 1], np.median([700 + n for n in range(30)]))

    def test_requires_thirty_samples_and_at_least_one_reference_zone(self):
        sampler = GroundSampler()
        invalid = np.zeros((8, 8), dtype=bool)
        for sequence in range(29):
            sampler.add(frame(sequence, np.full((8, 8), 700), invalid))
        self.assertIsNone(sampler.result())
        sampler.add(frame(29, np.full((8, 8), 700), invalid))
        self.assertIsNone(sampler.result())

    def test_insufficient_per_zone_samples_remain_missing(self):
        sampler = GroundSampler()
        for sequence in range(30):
            valid = np.ones((8, 8), dtype=bool)
            if sequence >= 23:
                valid[0, 0] = False
            if sequence >= 24:
                valid[0, 1] = False
            sampler.add(frame(sequence, np.full((8, 8), 700), valid))
        self.assertTrue(np.isnan(sampler.result()[0, 0]))
        self.assertTrue(np.isfinite(sampler.result()[0, 1]))

    def test_relative_display_masks_missing_reference_and_preserves_negative(self):
        reference = np.full((8, 8), 700.0)
        reference[0, 1] = np.nan
        current = frame(1, np.full((8, 8), 710))
        current.distance_mm[0, 2] = 690
        points, colors, grid = preview_data(current, reference)
        self.assertEqual(len(points), 63)
        self.assertEqual(grid[0, 1].tolist(), [0.22, 0.24, 0.27])
        np.testing.assert_allclose(points[0, 2], -0.1)
        np.testing.assert_allclose(colors[0], [0.12, 0.38, 0.68])

    def test_ground_capture_and_relative_display_use_absolute_one_metre_limit(self):
        sampler = GroundSampler()
        distance = np.full((8, 8), 1000)
        distance[0, 0] = 1001
        for sequence in range(30):
            sampler.add(frame(sequence, distance))
        reference = sampler.result()
        self.assertTrue(np.isnan(reference[0, 0]))
        self.assertEqual(reference[0, 1], 1000)
        current = frame(31, np.full((8, 8), 999))
        current.distance_mm[0, 2] = 1001
        values, valid = display_data(current, reference)
        self.assertEqual(values[0, 1], 1)
        self.assertFalse(valid[0, 0])
        self.assertFalse(valid[0, 2])
        self.assertEqual(valid.sum(), 62)


if __name__ == "__main__":
    unittest.main()
