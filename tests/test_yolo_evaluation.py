from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from evaluate_yolo_boards import mask_ious,assess


class EvaluationTests(unittest.TestCase):
    def test_exact_masks_and_empty_predictions(self):
        a=np.zeros((20,30),bool);a[2:10,3:15]=True
        b=np.zeros_like(a);b[12:18,20:28]=True
        np.testing.assert_allclose(mask_ious([a,b],[b,a]),[[0,1],[1,0]])
        result=assess(mask_ious([a,b],[]),[0,1],[])
        self.assertEqual(result['missed_annotation_ids'],[0,1])

    def test_class_errors_and_duplicates_are_not_true_positives(self):
        result=assess(np.array([[1.,1.]]),[0],[0,0])
        self.assertEqual(result['per_class']['VARa'],{'tp':1,'fp':1,'fn':0})
        result=assess(np.array([[1.]]),[0],[2])
        self.assertEqual(result['wrong_class_pairs'],[[0,0]])
        self.assertEqual(result['per_class']['VARa']['fn'],1)
        self.assertEqual(result['per_class']['VARc']['fp'],1)

    def test_assignment_maximizes_valid_matches_and_empty_scene(self):
        result=assess(np.array([[.9,.6],[.8,.1]]),[0,0],[0,0])
        self.assertEqual(len(result['matches']),2)
        result=assess(np.empty((0,1)),[],[3])
        self.assertEqual(result['per_class']['hand']['fp'],1)


if __name__=='__main__':unittest.main()
