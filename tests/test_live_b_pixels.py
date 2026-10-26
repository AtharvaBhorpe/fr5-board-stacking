import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from b_worker_protocol import read_reply
from b_live_geometry import self_test as geometry_test
from live_b_pixels import fresh,worker_command
from read_live_b_pixels import handoff


class LivePixelsTests(unittest.TestCase):
    def packet(self):
        return {'status':'ok','coordinate_system':'raw_image_pixels',
                'frame_id':'scene_camera_optical_frame','source_timestamp_sec':10.,
                'source_stamp':{'sec':10,'nanosec':0},'snapshot_id':'s1','image_size':[1920,1080],
                'objects':[{'instance_id':'s1:1','candidate_id':'candidate_1','centre_px':[500,400],
                            'orientation_image_deg':30.,'outline_kind':'enclosing_rectangle_only',
                            'supported_pixel_corners':None}]}

    def test_geometry(self):geometry_test()

    def test_fresh_handoff_and_fallback_marked(self):
        result=handoff(self.packet(),10.5)
        self.assertEqual(result['objects'][0]['centre_px'],[500.,400.])
        self.assertEqual(result['objects'][0]['outline_kind'],'enclosing_rectangle_only')
        self.assertFalse(result['execution_allowed'])
        self.assertFalse(result['objects'][0]['instance_isolation_verified'])

    def test_stale_error_wrong_frame_and_bad_coordinates_rejected(self):
        base=self.packet()
        for key,value in [('status','stale'),('frame_id','base_link'),
                          ('coordinate_system','metres'),('source_timestamp_sec',0.),
                          ('source_timestamp_sec',20.)]:
            bad=copy.deepcopy(base);bad[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):handoff(bad,10.5)
        for coordinates in ([float('nan'),2],[-1,200],[1920,200]):
            bad=copy.deepcopy(base);bad['objects'][0]['centre_px']=coordinates
            with self.subTest(coordinates=coordinates),self.assertRaises(ValueError):handoff(bad,10.5)

    def test_snapshot_ids_are_not_tracks(self):
        p=self.packet();p['objects'].append(copy.deepcopy(p['objects'][0]))
        with self.assertRaisesRegex(ValueError,'Duplicate'):handoff(p,10.5)

    def test_freshness(self):
        self.assertTrue(fresh(self.packet(),10.5,2))
        self.assertFalse(fresh(self.packet(),13,2))
        self.assertFalse(fresh(None,10,2))

    def test_worker_protocol(self):
        for code,expected in [('print(\'{"ready":true}\',flush=True)',{'ready':True}),('pass','exited'),
                              ('import time;time.sleep(5)','timed out')]:
            with self.subTest(code=code),subprocess.Popen([sys.executable,'-c',code],
                    stdout=subprocess.PIPE,text=True) as p:
                if isinstance(expected,dict):self.assertEqual(read_reply(p,2),expected)
                else:
                    with self.assertRaisesRegex(RuntimeError,expected):read_reply(p,.2 if expected=='timed out' else 2)
                p.wait(timeout=3)

    def test_live_settings_use_committed_scene_config(self):
        settings=json.loads((ROOT/'config/pretrained_b_live.json').read_text())
        scene=Path(settings['scene_config'])
        self.assertEqual(scene.parts[0],'config')
        self.assertTrue((ROOT/scene).is_file())

    def test_worker_has_a_clean_gpu_environment(self):
        command=worker_command(ROOT,ROOT/'config/pretrained_b_live.json')
        self.assertIn('--clean-env',command)
        self.assertEqual(command[command.index('-e')+1],'perception-gpu')


if __name__=='__main__':unittest.main()
