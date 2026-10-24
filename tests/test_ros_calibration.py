"""Compatibility check against the installed upstream ROS calibrator."""

from pathlib import Path
import tomllib
import unittest

import cv2
import numpy as np

try:
    from camera_calibration.calibrator import ChessboardInfo, MonoCalibrator, Patterns
except ImportError:
    ChessboardInfo = None


@unittest.skipIf(ChessboardInfo is None, 'Run this integration check in the camera-ros environment.')
class RosCalibrationTest(unittest.TestCase):
    def test_standard_charuco_solver_recovers_known_intrinsics(self):
        with open(Path(__file__).parents[1] / 'config' / 'perception.toml', 'rb') as source:
            settings = tomllib.load(source)['calibration']['boards']['large']
        board = ChessboardInfo('charuco', settings['grid_squares_x'], settings['grid_squares_y'],
                              settings['square_length_m'], settings['marker_length_m'],
                              settings['dictionary'].removeprefix('DICT_').lower())
        calibrator = MonoCalibrator([board], pattern=Patterns.ChArUco)
        calibrator.size = (1280, 720)
        objects = board.charuco_board.getChessboardCorners()
        ids = np.arange(len(objects), dtype=np.int32).reshape(-1, 1)
        matrix = np.array([[900., 0., 640.], [0., 910., 360.], [0., 0., 1.]])
        rng = np.random.default_rng(42)
        observations = []
        for _ in range(20):
            rotation = rng.uniform(-.35, .35, 3)
            translation = np.array([rng.uniform(-.3, .05), rng.uniform(-.2, .05), rng.uniform(.8, 1.1)])
            pixels, _ = cv2.projectPoints(objects, rotation, translation, matrix, np.zeros(5))
            observations.append((pixels.astype(np.float32), ids.copy(), board))
        calibrator.cal_fromcorners(observations)
        np.testing.assert_allclose(calibrator.intrinsics, matrix, rtol=.001, atol=.1)
