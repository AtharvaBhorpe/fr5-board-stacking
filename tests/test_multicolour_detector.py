import copy
from pathlib import Path
import tomllib
import unittest

import cv2
import numpy as np

from fr5_perception import ClassicalDetector


class MulticolourTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).parents[1]
        with (root / "config/perception_multicolour.toml").open("rb") as source:
            self.config = tomllib.load(source)
        with (root / "config/perception.toml").open("rb") as source:
            self.original = tomllib.load(source)

    def scene(self, colour, width=108, height=80):
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(
            image, (100, 90), (100 + width, 90 + height), colour, -1)
        return image

    def candidates(self, image, config=None):
        report = ClassicalDetector(
            self.config if config is None else config).detect(image)
        self.assertFalse(report["execution_allowed"])
        return report["quad_candidates"]

    def test_different_colours(self):
        colours = [
            (20, 35, 180), (180, 35, 20), (30, 150, 30),
            (20, 180, 180), (20, 110, 210), (170, 30, 160),
            (180, 180, 20),
        ]
        for colour in colours:
            with self.subTest(bgr=colour):
                found = self.candidates(self.scene(colour))
                self.assertEqual(len(found), 1)
                self.assertEqual(
                    found[0]["appearance"]["median_rgb"],
                    list(colour[::-1]))

    def test_grey_shades_have_ordered_lightness(self):
        lightness = []
        for value in (70, 120, 220):
            found = self.candidates(self.scene((value,) * 3))
            self.assertEqual(len(found), 1)
            lightness.append(found[0]["appearance"]["median_lab"][0])
        self.assertTrue(lightness[0] < lightness[1] < lightness[2])

    def test_two_projected_sizes(self):
        for width, height in ((108, 80), (276, 204)):
            with self.subTest(width=width):
                found = self.candidates(
                    self.scene((180, 35, 20), width, height))
                self.assertEqual(len(found), 1)
                self.assertAlmostEqual(
                    found[0]["size"]["long_side_px"], width, delta=1)
                self.assertIsNone(found[0]["size"]["long_side_m"])

    def test_two_separated_boards(self):
        image = self.scene((20, 35, 180))
        cv2.rectangle(image, (350, 220), (500, 330), (180, 35, 20), -1)
        self.assertEqual(len(self.candidates(image)), 2)

    def test_internal_printing_preserves_candidate(self):
        image = self.scene((220, 220, 220), 220, 160)
        cv2.putText(image, "PRINT", (120, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.line(image, (120, 200), (280, 200), (0, 0, 180), 4)
        self.assertEqual(len(self.candidates(image)), 1)

    def test_white_patch_on_coloured_board(self):
        image = self.scene((180, 35, 20), 220, 160)
        cv2.rectangle(image, (140, 120), (180, 160), (255, 255, 255), -1)
        self.assertEqual(len(self.candidates(image)), 1)

    def test_dark_background_has_no_candidates(self):
        image = np.full((480, 640, 3), 20, np.uint8)
        self.assertEqual(self.candidates(image), [])

    def test_region_clipped_board_is_rejected(self):
        config = copy.deepcopy(self.config)
        config["classical"]["pickup_polygon_normalized"] = [
            [0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8]]
        self.assertEqual(
            self.candidates(self.scene((180, 35, 20)), config), [])

    def test_invalid_hsv_range_is_rejected(self):
        config = copy.deepcopy(self.config)
        config["classical"]["foreground_hsv_ranges"] = [
            [190, 0, 0, 200, 255, 255]]
        with self.assertRaises(ValueError):
            self.candidates(self.scene((180, 35, 20)), config)

    def test_original_white_profile_remains_unchanged(self):
        self.assertEqual(
            self.candidates(self.scene((180, 35, 20)), self.original), [])
        self.assertEqual(
            len(self.candidates(self.scene((220, 220, 220)), self.original)),
            1)
