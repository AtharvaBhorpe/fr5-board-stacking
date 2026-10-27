import importlib.util
import json
from pathlib import Path
import unittest
import sys
import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('sorting_geometry', Path(__file__).resolve().parents[1]/'scripts/sorting_geometry.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from configure_bin_offsets import placement_offset


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads((Path(__file__).resolve().parents[1]/'config/sorting_task.json').read_text())
        t = np.diag([1., -1., -1., 1.]); t[2, 3] = 1
        self.cal = {'T_base_camera': t.tolist(), 'camera_matrix': [[800., 0, 320], [0, 800., 240], [0, 0, 1.]],
                    'distortion_coefficients': [0.]*5}

    def test_projection_and_pose(self):
        xy = np.array([[-.1475, -.105], [.1475, -.105], [.1475, .105], [-.1475, .105]])
        uv = xy * [800, -800] + [320, 240]
        xyz = g.rays_to_plane(uv, self.cal, [0, 0, 0], [0, 0, 1])
        np.testing.assert_allclose(xyz[:, :2], xy)
        pose, dimensions = g.surface_pose(xyz, [0., 0., 1.])
        np.testing.assert_allclose(pose[:3, 3], [0, 0, 0], atol=1e-12)
        self.assertEqual(g.classify(dimensions, 'white', self.settings), 'VARa')

    def test_classes_and_rejection(self):
        self.assertEqual(g.classify([.210, .152], 'white', self.settings), 'VARb')
        self.assertEqual(g.classify([.210, .142], 'red', self.settings), 'VARc')
        with self.assertRaises(ValueError): g.classify([.42, .15], 'white', self.settings)
        with self.assertRaises(ValueError): g.classify([.21, .15], 'unknown', self.settings)

    def test_colour(self):
        im = np.full((100, 100, 3), (0, 0, 180), np.uint8)
        mask = np.ones((100, 100), bool)
        self.assertEqual(g.colour(im, mask, self.settings)[0], 'red')
        im[:] = 190
        self.assertEqual(g.colour(im, mask, self.settings)[0], 'white')

    def test_occluded_outline_rejected(self):
        mask = np.zeros((200, 200), np.uint8)
        mask[20:160, 20:160] = 255
        self.assertEqual(g.outline(mask, self.settings).shape, (4, 2))
        mask[60:120, 100:180] = 0
        with self.assertRaises(ValueError): g.outline(mask, self.settings)

    def test_bad_transforms_and_plane(self):
        with self.assertRaises(ValueError): g.rigid(np.zeros((4, 4)))
        with self.assertRaises(ValueError): g.rays_to_plane([[320, 240]], self.cal, [0, 0, 0], [0, 0, 0])
        with self.assertRaises(ValueError): g.rays_to_plane([[320, 240]], self.cal, [0, 0, 2], [0, 0, 1])

    def test_missing_tag_geometry(self):
        self.settings['bins']['1']['tag_size_m'] = None
        image = np.full((480, 640, 3), 255, np.uint8)
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), 1, 100)
        image[180:280, 260:360] = marker[:, :, None]
        bins = g.bin_observations(image, self.settings, self.cal)
        self.assertFalse(any(b['pose_available'] for b in bins))
        self.assertIn('Measured tag size', bins[0]['reasons'][0])

    def test_synthetic_tag_pose(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), 1, 100)
        image[180:280, 260:360] = marker[:, :, None]
        self.settings['bins']['1'] = {'tag_size_m': .05, 'T_tag_placement': np.eye(4).tolist()}
        self.settings['maximum_tag_rms_px'] = 2
        result = g.bin_observations(image, self.settings, self.cal)[0]
        self.assertTrue(result['pose_available'], result)
        self.assertFalse(result['execution_allowed'])

    def test_bin_click_direction(self):
        camera_tag = np.diag([1., -1., -1., 1.]); camera_tag[2, 3] = 1
        offset = placement_offset(camera_tag, [400, 160], [480, 160], .1385, self.cal)
        np.testing.assert_allclose(offset[:3, :3], np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.norm(offset[:3, 3]), .1385)
        self.assertTrue(np.all(offset[:2, 3] > 0))
        with self.assertRaises(ValueError):
            placement_offset(camera_tag, [400, 160], [400, 160], .1385, self.cal)


if __name__ == '__main__': unittest.main()
