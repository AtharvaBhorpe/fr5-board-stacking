import copy
from pathlib import Path
import tomllib
import unittest

import cv2
import numpy as np

from fr5_perception import ClassicalDetector
from fr5_perception.detector import region_mask


class ClassicalDetectorTest(unittest.TestCase):
    def setUp(self):
        with open(Path(__file__).parents[1] / 'config' / 'perception.toml', 'rb') as source:
            self.config = tomllib.load(source)

    def test_empty_table_has_no_candidates(self):
        report = ClassicalDetector(self.config).detect(np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(report['quad_candidates'], [])
        self.assertFalse(report['execution_allowed'])

    def test_rotated_white_board_is_a_pixel_candidate_only(self):
        image = np.zeros((480, 640, 3), np.uint8)
        corners = np.rint(cv2.boxPoints(((300, 230), (220, 150), 25))).astype(np.int32)
        cv2.fillConvexPoly(image, corners, (230, 230, 230))
        report = ClassicalDetector(self.config).detect(image)
        self.assertEqual(len(report['quad_candidates']), 1)
        candidate = report['quad_candidates'][0]
        np.testing.assert_allclose(candidate['pixel_center'], [300, 230], atol=2)
        self.assertIsNone(candidate['metric_pose'])
        self.assertFalse(report['execution_allowed'])

    def test_board_cut_by_region_is_rejected(self):
        config = copy.deepcopy(self.config)
        config['classical']['pickup_polygon_normalized'] = [[0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8]]
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(image, (80, 150), (300, 320), (255, 255, 255), -1)
        report = ClassicalDetector(config).detect(image)
        self.assertEqual(report['quad_candidates'], [])

    def test_tagged_board_retains_outer_boundary(self):
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(image, (150, 130), (420, 330), (255, 255, 255), -1)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        tag = cv2.aruco.generateImageMarker(dictionary, 3, 70)
        image[190:260, 250:320] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
        report = ClassicalDetector(self.config).detect(image)
        self.assertEqual(len(report['quad_candidates']), 1)
        self.assertTrue(any(marker['id'] == 3 for marker in report['quad_candidates'][0]['contained_markers']))

    def test_duplicate_marker_ids_are_explicit(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        tag = cv2.cvtColor(cv2.aruco.generateImageMarker(dictionary, 3, 100), cv2.COLOR_GRAY2BGR)
        image[150:250, 100:200] = tag
        image[150:250, 350:450] = tag
        report = ClassicalDetector(self.config).detect(image)
        self.assertIn({'dictionary_used': 'DICT_5X5_100', 'id': 3, 'count': 2}, report['duplicate_marker_ids'])
        self.assertFalse(report['calibration_valid'])

    def test_invalid_region_and_image_fail(self):
        with self.assertRaises(ValueError):
            region_mask((480, 640, 3), [[-1, 0], [1, 0], [1, 1]])
        with self.assertRaises(ValueError):
            ClassicalDetector(self.config).detect(np.zeros((480, 640), np.uint8))


if __name__ == '__main__':
    unittest.main()
