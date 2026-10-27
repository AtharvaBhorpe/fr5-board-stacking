import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from capture_board_dataset import save_capture
from check_capture_coverage import inspect_sample,layout_signature


class CoverageTests(unittest.TestCase):
    def test_legacy_layout_can_match_but_changed_board_cannot(self):
        scene={'id':13,'boards':[{'class':'VARa','x':100,'y':100,'angle':0}],'occluder':None,'effect':None}
        updated={**scene,'guide_revision':'v2','priority':True}
        self.assertEqual(layout_signature(scene),layout_signature(updated))
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);metadata={'source_timestamp_sec':10.,'saved_ros_time_sec':10.1,'intended_guide_scene':scene}
            folder,_=save_capture(root,np.zeros((40,60,3),np.uint8),metadata)
            self.assertTrue(inspect_sample(folder,{13:updated})['matches_current_intended_layout'])
            changed={**updated,'boards':[]}
            self.assertFalse(inspect_sample(folder,{13:changed})['matches_current_intended_layout'])

    def test_corrupt_saved_image_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);folder,metadata=save_capture(root,np.zeros((40,60,3),np.uint8),{'source_timestamp_sec':10.,'saved_ros_time_sec':10.1})
            (folder/metadata['image_file']).write_bytes(b'broken')
            with self.assertRaisesRegex(ValueError,'checksum'):inspect_sample(folder,{})


if __name__=='__main__':unittest.main()
