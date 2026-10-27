import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest
import tempfile
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from b_worker_protocol import read_reply
from b_live_geometry import self_test as geometry_test
from live_b_pixels import fresh,worker_command,read_frame,write_frame
from read_live_b_pixels import handoff


class LivePixelsTests(unittest.TestCase):
    def test_lossless_frame_transfer_and_bad_format_rejection(self):
        image=np.random.default_rng(7).integers(0,256,(48,64,3),dtype=np.uint8)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'frame.npy'
            write_frame(path,image)
            self.assertTrue(np.array_equal(read_frame(path),image))
            write_frame(path,np.zeros((48,64),dtype=np.uint8))
            with self.assertRaises(ValueError):read_frame(path)

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

    def test_timestamp_angle_identity_confidence_and_uncertainty_rejected(self):
        for change in ('timestamp','nanoseconds','angle','identity','confidence','uncertain'):
            packet=self.packet();obj=packet['objects'][0]
            if change=='timestamp':packet['source_stamp']['sec']=11
            if change=='nanoseconds':packet['source_stamp']['nanosec']=1_000_000_000
            if change=='angle':obj['orientation_image_deg']=180
            if change=='identity':obj['instance_id']=''
            if change=='confidence':obj['confidence']=float('nan')
            if change=='uncertain':obj['detection_status']='uncertain'
            with self.subTest(change=change),self.assertRaises(ValueError):handoff(packet,10.5)

    def test_empty_frame_contract_and_angle_units(self):
        packet=self.packet();packet['objects']=[]
        result=handoff(packet,10.5)
        self.assertEqual(result['objects'],[])
        self.assertEqual(result['units']['centre_px'],'pixels')
        self.assertFalse(result['execution_allowed'])

    def test_partial_worker_reply_cannot_hang_or_desynchronize(self):
        code='import sys,time;sys.stdout.write(\'{"ok":\');sys.stdout.flush();time.sleep(5)'
        with subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True) as process:
            started=time.monotonic()
            with self.assertRaisesRegex(RuntimeError,'timed out'):read_reply(process,.2)
            process.wait(timeout=2)
            self.assertLess(time.monotonic()-started,2)

    def test_invalid_worker_json_and_nonobject_fail_closed(self):
        for code in ('print("bad",flush=True)','print("[]",flush=True)'):
            with self.subTest(code=code),subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True) as process:
                with self.assertRaises(ValueError):read_reply(process,2)
                process.wait(timeout=2)

    def test_uncertain_previews_are_not_accepted_coordinates(self):
        p=self.packet();p['uncertain_objects']=[copy.deepcopy(p['objects'][0])]
        result=handoff(p,10.5)
        self.assertEqual(result['uncertain_count'],1)
        self.assertEqual(len(result['objects']),1)

    def test_uncertain_geometry_remains_a_fallback(self):
        from b_live_geometry import measure
        candidate={'id':'uncertain_1','pixel_corners':[[10,10],[80,10],[80,50],[10,50]],
                   'pixel_center':[45,30],'detection_status':'uncertain'}
        item=measure(candidate)
        self.assertIsNone(item['supported_pixel_corners'])
        self.assertEqual(item['detection_status'],'uncertain')

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
