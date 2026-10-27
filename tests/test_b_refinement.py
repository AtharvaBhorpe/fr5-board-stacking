import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from b_refinement import rectangle_check,interior_seeds,prompts_for_masks,select_consistent


class RefinementTests(unittest.TestCase):
    def scene(self):
        image=np.zeros((240,320,3),np.uint8)
        mask=np.zeros((240,320),np.uint8);mask[60:180,80:250]=1
        image[mask>0]=220
        return image,mask.astype(bool)

    def test_visible_rectangle_passes_but_unsupported_outline_fails(self):
        image,mask=self.scene()
        self.assertTrue(rectangle_check(image,mask)['passed'])
        self.assertFalse(rectangle_check(np.zeros_like(image),mask)['passed'])

    def test_cross_union_rejected(self):
        image,mask=self.scene();mask[20:220,140:190]=True;image[mask]=220
        self.assertFalse(rectangle_check(image,mask)['passed'])

    def test_visible_internal_seam_rejects_merged_rectangle(self):
        image,mask=self.scene()
        cv2.line(image,(165,60),(165,179),(60,60,60),2)
        self.assertFalse(rectangle_check(image,mask)['passed'])
        prompts=prompts_for_masks([mask],[{'passed':False}],6,image=image)
        self.assertTrue(any(p.get('evidence')=='observed_internal_line' for p in prompts))

    def test_interior_prompts_are_inside_and_exclusions_outside(self):
        _,a=self.scene();b=np.roll(a,100,axis=1)
        prompts=prompts_for_masks([a,b],[{'passed':False}]*2,4)
        self.assertLessEqual(len(prompts),4)
        for p in prompts:
            mask=[a,b][p['source_index']]
            for point,label in zip(p['points'],p['labels']):
                self.assertEqual(bool(mask[point[1],point[0]]),bool(label))
        self.assertEqual(interior_seeds(np.zeros_like(a)),[])

    def test_duplicates_do_not_become_two_instances(self):
        image,mask=self.scene()
        candidates=[{'mask':mask,'quality':.9},{'mask':mask.copy(),'quality':.8}]
        accepted,rejected=select_consistent(image,candidates)
        self.assertEqual(len(accepted),1);self.assertEqual(len(rejected),1)

    def test_two_separated_instances_remain_two(self):
        image,a=self.scene();b=np.zeros_like(a);b[10:45,10:65]=True;image[b]=220
        accepted,_=select_consistent(image,[{'mask':a,'quality':.9},{'mask':b,'quality':.8}])
        self.assertEqual(len(accepted),2)

    def test_perpendicular_top_board_and_exposed_bottom_are_not_merged(self):
        image=np.zeros((260,320,3),np.uint8)
        bottom=np.zeros((260,320),np.uint8);bottom[100:160,30:290]=1
        top=np.zeros_like(bottom);top[30:230,125:195]=1
        image[bottom>0]=210;image[top>0]=230
        visible_bottom=(bottom>0) & ~(top>0)
        selected,_=select_consistent(image,[{'mask':(bottom|top)>0,'quality':.99},
            {'mask':top>0,'quality':.95},{'mask':visible_bottom,'quality':.9}])
        self.assertEqual(len(selected),1)
        self.assertTrue(np.array_equal(selected[0]['mask'],top>0))

if __name__=='__main__':unittest.main()
