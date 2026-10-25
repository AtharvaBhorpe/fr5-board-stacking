from pathlib import Path
import tomllib
import unittest

import cv2
import numpy as np

from fr5_perception import ClassicalDetector
from fr5_perception.measurements import describe_region


class MeasurementTest(unittest.TestCase):
    def sample(self, colour, width=100):
        image = np.zeros((300, 400, 3), np.uint8)
        mask = np.zeros(image.shape[:2], np.uint8)
        cv2.rectangle(mask, (40, 40), (40 + width, 120), 255, -1)
        image[mask > 0] = colour
        return image, mask

    def test_red_and_blue_measurements(self):
        for bgr in [(0, 0, 220), (220, 0, 0)]:
            with self.subTest(bgr=bgr):
                result = describe_region(*self.sample(bgr))
                self.assertEqual(
                    result["appearance"]["median_rgb"], list(bgr[::-1]))
                self.assertIsNone(result["appearance"]["colour_label"])

    def test_light_and_dark_shades(self):
        dark = describe_region(*self.sample((90, 90, 90)))
        light = describe_region(*self.sample((210, 210, 210)))
        self.assertGreater(
            light["appearance"]["median_lab"][0],
            dark["appearance"]["median_lab"][0])

    def test_pixel_size_is_not_metric_size(self):
        small = describe_region(*self.sample((220, 220, 220), 100))
        large = describe_region(*self.sample((220, 220, 220), 200))
        self.assertAlmostEqual(small["size"]["long_side_px"], 100)
        self.assertAlmostEqual(large["size"]["long_side_px"], 200)
        self.assertIsNone(large["size"]["long_side_m"])

    def test_a_emits_measurements(self):
        config_path = Path(__file__).parents[1] / "config/perception.toml"
        with config_path.open("rb") as source:
            config = tomllib.load(source)
        image, _ = self.sample((220, 220, 220))
        report = ClassicalDetector(config).detect(image)
        self.assertEqual(len(report["quad_candidates"]), 1)
        candidate = report["quad_candidates"][0]
        self.assertEqual(
            candidate["appearance"]["median_rgb"], [220, 220, 220])
        self.assertIn("size", candidate)
        self.assertFalse(report["execution_allowed"])

    def test_empty_mask_rejected(self):
        image, mask = self.sample((220, 220, 220))
        with self.assertRaises(ValueError):
            describe_region(image, np.zeros_like(mask))
