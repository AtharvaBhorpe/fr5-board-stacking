import copy
import json
import tomllib
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from yolo_pixels import class_names,translate_predictions
from b_live_geometry import measure
from read_live_b_pixels import handoff


class YoloAdapterTests(unittest.TestCase):
    def test_live_full_table_accepts_both_halves_and_keeps_pixel_geometry(self):
        root=Path(__file__).resolve().parents[1]
        settings=json.loads((root/'config/yolo_live.json').read_text())
        self.assertEqual(settings['scene_config'],'config/perception_full_table.toml')
        config=tomllib.loads((root/settings['scene_config']).read_text())
        image=np.full((240,400,3),30,np.uint8)
        masks=np.zeros((4,240,400),bool)
        for mask,(y,x) in zip(masks,[(30,30),(30,270),(150,30),(150,270)]):
            mask[y:y+45,x:x+70]=True;image[mask]=220
        report,accepted,_,roi=translate_predictions(image,masks,np.zeros(4),np.full(4,.9),
                                                   ['VARa','VARb','VARc','hand'],config)
        self.assertTrue(np.all(roi==255));self.assertEqual(len(accepted),4)
        self.assertFalse(report['rejected_proposals'])
        for (candidate,mask),(y,x) in zip(accepted,[(30,30),(30,270),(150,30),(150,270)]):
            measured=measure(candidate,mask)
            np.testing.assert_allclose(measured['centre_px'],[x+34.5,y+22],atol=1)
            self.assertLess(min(measured['orientation_image_deg'],180-measured['orientation_image_deg']),1)

    def test_full_table_does_not_accept_image_clipped_board(self):
        root=Path(__file__).resolve().parents[1]
        config=tomllib.loads((root/'config/perception_full_table.toml').read_text())
        image=np.full((240,400,3),220,np.uint8);mask=np.zeros((1,240,400),bool)
        mask[0,40:100,:70]=True
        report,accepted,_,_=translate_predictions(image,mask,np.array([0]),np.array([.9]),
                                                 ['VARa','VARb','VARc','hand'],config)
        self.assertFalse(accepted)
        self.assertIn('boundary',report['rejected_proposals'][0]['reason'])

    def config(self):return {'classical':{'pickup_polygon_normalized':[],
        'min_area_fraction':.001,'max_area_fraction':.8}}

    def inputs(self):
        image=np.zeros((240,400,3),np.uint8)
        masks=np.zeros((3,240,400),bool)
        masks[0,30:110,30:140]=True;masks[1,130:210,190:300]=True
        masks[2,0:20,0:30]=True
        for mask in masks:image[mask]=220
        return image,masks,np.array([0,0,3]),np.array([.9,.85,.8])

    def test_same_class_instances_distinct_and_hand_separate(self):
        report,accepted,_,_=translate_predictions(*self.inputs(),
            ['large_white','small_white','small_red','hand'],self.config())
        self.assertEqual(len(accepted),2)
        self.assertNotEqual(accepted[0][0]['id'],accepted[1][0]['id'])
        self.assertEqual(len(report['hand_detections']),1)
        self.assertTrue(all(item['class_name']=='VARa' for item,_ in accepted))
        obj=measure(*accepted[0]);self.assertEqual(obj['class_name'],'VARa')
        np.testing.assert_allclose(obj['centre_px'],[84.5,69.5],atol=1)
        self.assertEqual(report['image_size'],[400,240])

    def test_hands_not_filtered_by_pickup_region(self):
        config=self.config();config['classical']['pickup_polygon_normalized']=[[.4,.4],[.9,.4],[.9,.95],[.4,.95]]
        report,_,_,_=translate_predictions(*self.inputs(),
            ['large_white','small_white','small_red','hand'],config)
        self.assertEqual(len(report['hand_detections']),1)
        self.assertTrue(report['rejected_proposals'])

    def test_wrong_resolution_and_nan_rejected(self):
        image,masks,classes,scores=self.inputs();names=['large_white','small_white','small_red','hand']
        with self.assertRaisesRegex(ValueError,'resolution'):
            translate_predictions(image,masks[:,:100],classes,scores,names,self.config())
        scores[0]=np.nan
        with self.assertRaisesRegex(ValueError,'Non-finite'):
            translate_predictions(image,masks,classes,scores,names,self.config())

    def test_bad_checkpoint_names_rejected(self):
        with self.assertRaises(ValueError):class_names(['person','book'])

    def test_renamed_and_legacy_checkpoints_emit_same_canonical_names(self):
        for names in (['VARa','VARb','VARc','hand'],['large_white','small_white','small_red','hand']):
            report,accepted,_,_=translate_predictions(*self.inputs(),names,self.config())
            self.assertEqual(accepted[0][0]['class_name'],'VARa')
            self.assertEqual(accepted[0][0]['class_id'],0)
            self.assertEqual(report['hand_detections'][0]['class_id'],3)

    def test_reordered_model_class_ids_remap_to_stable_interface_ids(self):
        report,accepted,_,_=translate_predictions(*self.inputs(),
            ['VARc','VARa','VARb','hand'],self.config())
        self.assertEqual(accepted[0][0]['class_name'],'VARc')
        self.assertEqual(accepted[0][0]['class_id'],2)

    def test_empty_frame_is_valid_observation_not_safety_clearance(self):
        image=np.zeros((240,400,3),np.uint8)
        report,accepted,_,_=translate_predictions(image,np.empty((0,240,400)),np.empty(0),np.empty(0),
            ['large_white','small_white','small_red','hand'],self.config())
        self.assertFalse(accepted);self.assertFalse(report['execution_allowed'])

    def test_nonquad_remains_visible_as_uncertain(self):
        image,masks,classes,scores=self.inputs();masks[0,65:110,80:140]=False
        report,_,_,_=translate_predictions(image,masks,classes,scores,
            ['large_white','small_white','small_red','hand'],self.config())
        self.assertEqual(len(report['uncertain_candidates']),1)

    def test_handoff_rejects_hand_in_board_array(self):
        packet={'status':'ok','coordinate_system':'raw_image_pixels','frame_id':'scene_camera_optical_frame',
                'source_timestamp_sec':10.,'source_stamp':{'sec':10,'nanosec':0},'snapshot_id':'s',
                'image_size':[400,240],'objects':[{'class_name':'hand'}]}
        with self.assertRaisesRegex(ValueError,'Hands'):handoff(packet,10.5)

if __name__=='__main__':unittest.main()
