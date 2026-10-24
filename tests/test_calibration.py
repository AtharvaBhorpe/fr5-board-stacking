import copy
from pathlib import Path
import tomllib
import unittest

import cv2
import numpy as np

from fr5_perception.calibration import board_from_config, fit_intrinsics, observe


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        with open(Path(__file__).parents[1] / 'config' / 'perception.toml', 'rb') as source:
            self.config = tomllib.load(source)

    def test_known_grid_finds_all_internal_corners(self):
        board = board_from_config(self.config)
        gray = board.generateImage((1000, 750), marginSize=30)
        objects, pixels, ids = observe(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), self.config)
        self.assertEqual(len(ids), 24)
        self.assertEqual(len(np.unique(ids)), 24)
        self.assertEqual(objects.shape, (24, 3))

    def test_unmeasured_grid_is_rejected(self):
        config = copy.deepcopy(self.config)
        config['calibration']['boards'][config['calibration']['active_board']]['square_length_m'] = 0
        with self.assertRaisesRegex(ValueError, 'Measure'):
            board_from_config(config)

    def test_each_print_uses_its_own_geometry(self):
        large = board_from_config(self.config, 'large').getChessboardCorners()
        small = board_from_config(self.config, 'small').getChessboardCorners()
        self.assertAlmostEqual(float(large[0, 0]), .052, places=6)
        self.assertAlmostEqual(float(small[0, 0]), .032, places=6)
        self.assertNotEqual(float(large[-1, 0]), float(small[-1, 0]))

    def test_insufficient_views_cannot_produce_a_calibration(self):
        with self.assertRaisesRegex(ValueError, 'Need at least'):
            fit_intrinsics([], None, self.config['calibration'])

    def test_projected_mixed_grid_views_recover_known_intrinsics(self):
        matrix = np.array([[900., 0., 640.], [0., 910., 360.], [0., 0., 1.]])
        rng = np.random.default_rng(42)
        observations = []
        for index in range(20):
            board = board_from_config(self.config, 'large' if index % 2 else 'small')
            objects = board.getChessboardCorners().astype(np.float32)
            rotation = rng.uniform(-.3, .3, 3)
            translation = np.array([rng.uniform(-.4, .12), rng.uniform(-.27, .08), rng.uniform(.8, .95)])
            pixels, _ = cv2.projectPoints(objects, rotation, translation, matrix, np.zeros(5))
            observations.append((objects, pixels.astype(np.float32)))
        settings = copy.deepcopy(self.config['calibration'])
        settings['min_axis_coverage_fraction'] = .3
        report = fit_intrinsics(observations, (1280, 720), settings)
        np.testing.assert_allclose(report['camera_matrix'], matrix, rtol=.001, atol=.1)
        self.assertTrue(report['intrinsic_checks_passed'], report['warnings'])
        self.assertFalse(report['metric_workcell_calibrated'])
        self.assertFalse(report['execution_allowed'])
        self.assertEqual(len(report['held_out_rms_px']), 4)
        settings['min_tilt_spread_deg'] = 180
        rejected = fit_intrinsics(observations, (1280, 720), settings)
        self.assertFalse(rejected['intrinsic_checks_passed'])


if __name__ == '__main__':
    unittest.main()
