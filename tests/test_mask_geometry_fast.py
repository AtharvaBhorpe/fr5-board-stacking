from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from b_live_geometry import supported_quad,quad_center_and_axis
from inspect_b import describe_mask
from fr5_perception.measurements import describe_region


class FastGeometryTests(unittest.TestCase):
    def scene(self):
        image=np.full((500,800,3),20,np.uint8)
        mask=np.zeros(image.shape[:2],np.uint8);mask[170:290,370:550]=1
        image[mask>0]=(20,40,200)
        roi=np.full(mask.shape,255,np.uint8)
        return image,mask.astype(bool),roi,{'min_area_fraction':.001,'max_area_fraction':.8}

    def test_cropped_measurements_keep_global_coordinates_and_colours(self):
        image,mask,roi,settings=self.scene()
        expected=describe_region(image,mask.astype(np.uint8)*255)
        result=describe_mask(image,mask,roi,settings)
        np.testing.assert_allclose(result['pixel_center'],[459.5,229.5])
        self.assertEqual(result['appearance'],expected['appearance'])
        expected_corners=cv2.boxPoints(((459.5,229.5),(119.,179.),90.))
        np.testing.assert_allclose(sorted(map(tuple,result['pixel_corners'])),sorted(map(tuple,expected_corners)),atol=1e-4)
        fast=describe_mask(image,mask,roi,settings,measure_appearance=False)
        self.assertIsNone(fast['appearance'])
        np.testing.assert_allclose(fast['pixel_corners'],result['pixel_corners'])
        self.assertEqual(fast['pixel_center'],result['pixel_center'])

    def test_tiny_notch_recovered_without_reducing_overlap_requirement(self):
        image,mask,roi,settings=self.scene()
        mask[170:178,440:450]=False
        contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        previous=cv2.approxPolyDP(contours[0],.01*cv2.arcLength(contours[0],True),True)
        self.assertNotEqual(len(previous),4)
        q=supported_quad(mask)
        self.assertIsNotNone(q)
        centre,angle=quad_center_and_axis(q)
        np.testing.assert_allclose(centre,[459.5,229.5],atol=1)
        predicted=np.zeros(mask.shape,np.uint8);cv2.fillConvexPoly(predicted,q.astype(np.int32),1)
        self.assertGreaterEqual(np.count_nonzero(mask&(predicted>0))/np.count_nonzero(mask|(predicted>0)),.93)

    def test_tiny_satellite_allowed_but_multiple_board_components_rejected(self):
        _,mask,_,_=self.scene()
        mask[150:152,360:362]=True
        self.assertIsNotNone(supported_quad(mask))
        mask[50:100,100:160]=True
        self.assertIsNone(supported_quad(mask))

    def test_large_occlusion_cross_empty_and_hole_remain_uncertain(self):
        _,mask,_,_=self.scene()
        notch=mask.copy();notch[170:230,420:500]=False
        self.assertIsNone(supported_quad(notch))
        cross=mask.copy();cross[110:350,430:480]=True
        self.assertIsNone(supported_quad(cross))
        hole=mask.copy();hole[190:270,390:530]=False
        self.assertIsNone(supported_quad(hole))
        self.assertIsNone(supported_quad(np.zeros_like(mask)))

    def test_cropped_components_still_report_fragmentation(self):
        image,mask,roi,settings=self.scene();mask[220:230,370:550]=False
        result=describe_mask(image,mask,roi,settings)
        self.assertEqual(result['visible_components'],2)
        self.assertIsNone(supported_quad(mask))

    def test_cached_outline_must_be_finite_and_convex(self):
        from b_live_geometry import measure
        image,mask,roi,settings=self.scene()
        candidate=describe_mask(image,mask,roi,settings);candidate['id']='one'
        candidate['supported_mask_quad']=[[1,1],[2,2],[1,2],[2,1]]
        with self.assertRaisesRegex(ValueError,'cached'):
            measure(candidate)


if __name__=='__main__':unittest.main()
