import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from capture_board_dataset import save_capture,nearest,finite_json


class CaptureTests(unittest.TestCase):
    def image(self):return np.random.default_rng(10).integers(0,256,(48,64,3),dtype=np.uint8)
    def meta(self):return {'saved_ros_time_sec':10.,'source_timestamp_sec':9.8,'joint_state':{'velocity':[float('nan')]}}

    def test_original_pixels_and_metadata_saved(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);image=self.image();target,meta=save_capture(root,image,self.meta())
            saved=root/'images'/meta['image_file']
            np.testing.assert_array_equal(cv2.imread(str(saved)),image)
            self.assertEqual(meta['image_sha256'],hashlib.sha256(saved.read_bytes()).hexdigest())
            stored=json.loads((target/'metadata.json').read_text())
            self.assertEqual(stored['joint_state']['velocity'],[None])
            self.assertTrue(stored['annotations_pending']);self.assertFalse(stored['execution_allowed'])
            self.assertEqual(stored['classes_dimensions_mm']['VARa'],[295,210,4])

    def test_stale_future_and_bad_images_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            for stamp in (0.,11.,float('nan')):
                m=self.meta();m['source_timestamp_sec']=stamp
                with self.assertRaises(ValueError):save_capture(Path(d),self.image(),m)
            with self.assertRaises(ValueError):save_capture(Path(d),np.zeros((48,64),np.uint8),self.meta())
            self.assertFalse(list(Path(d).glob('.pending*')))

    def test_optional_raw_depth_preserved_and_names_unique(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);depth=np.arange(48*64,dtype=np.uint16).reshape(48,64)
            first,a=save_capture(root,self.image(),self.meta(),{'wrist_depth':depth})
            second,b=save_capture(root,self.image(),self.meta())
            self.assertNotEqual(a['capture_id'],b['capture_id'])
            np.testing.assert_array_equal(np.load(first/'wrist_depth.npy',allow_pickle=False),depth)
            self.assertEqual(len(list((root/'images').glob('*.png'))),2)

    def test_bad_metadata_does_not_leave_partial_capture(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);metadata=self.meta();metadata['bad']=object()
            with self.assertRaises(TypeError):save_capture(root,self.image(),metadata)
            self.assertFalse(list(root.glob('.pending*')))
            self.assertFalse(list((root/'images').glob('*.png')))

    def test_optional_stream_matching_requires_near_timestamp(self):
        def message(seconds):return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(seconds),nanosec=round((seconds-int(seconds))*1e9))))
        values=[message(9.),message(10.),message(10.2)]
        self.assertIs(nearest(values,10.01),values[1])
        self.assertIsNone(nearest(values,12.))
        self.assertIsNone(nearest([],10.))


if __name__=='__main__':unittest.main()
