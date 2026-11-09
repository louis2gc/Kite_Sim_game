"""Projection checks against physical directions, independent of scene artwork."""
import unittest
from kite_sim.app.camera import Camera


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera((1150, 820))

    def test_horizon_stays_level_across_azimuths(self):
        for azimuth in (-80, -45, 0, 45, 80):
            point = self.camera.direction(azimuth, 0)
            self.assertAlmostEqual(point[1], self.camera.horizon)

    def test_settling_from_overhead_moves_down_screen(self):
        overhead = self.camera.direction(0, 85)
        settled = self.camera.direction(0, 77.5)
        self.assertAlmostEqual(overhead[0], 575)
        self.assertLess(overhead[1], settled[1])

    def test_left_right_mirror_and_behind_camera(self):
        left = self.camera.direction(-40, 30)
        right = self.camera.direction(40, 30)
        self.assertLess(left[0], 575)
        self.assertGreater(right[0], 575)
        self.assertAlmostEqual(left[0]+right[0], 1150)
        self.assertAlmostEqual(left[1], right[1])
        self.assertIsNone(self.camera.direction(180, 0))
