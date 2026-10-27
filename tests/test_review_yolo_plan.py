import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prepare_yolo_pilot import NAMES,digest
from review_yolo_plan import load_plan,reviewed_plan


class ReviewPlanTests(unittest.TestCase):
    def plan(self):
        return {'source_zip_sha256':'original','target_split':{'train':27,'val':9},
            'images':[{'file_name':f'{i}.png','exported_instances':1,'reviewed':False,
                'arrangement_group':'','split':'','verified_empty':False} for i in range(4)]}

    def edits(self):
        rows=copy.deepcopy(self.plan()['images'])
        for i,row in enumerate(rows):
            row.update(reviewed=i<3,arrangement_group='layout_a' if i<2 else 'layout_b',
                       split='train' if i<2 else 'val')
        return rows

    def test_bulk_review_preserves_source_and_excludes_pending(self):
        original=self.plan();edits=self.edits();edits[0]['exported_instances']=999
        result=reviewed_plan(original,edits)
        self.assertEqual(result['target_split'],{'train':2,'val':1})
        self.assertEqual(result['source_zip_sha256'],'original')
        self.assertEqual(result['images'][0]['exported_instances'],1)
        self.assertFalse(result['images'][3]['reviewed'])
        self.assertFalse(original['images'][0]['reviewed'])

    def test_arrangement_leakage_rejected(self):
        edits=self.edits();edits[2]['arrangement_group']='layout_a'
        with self.assertRaisesRegex(ValueError,'both|Same arrangement'):reviewed_plan(self.plan(),edits)

    def test_blank_annotations_require_explicit_confirmation(self):
        original=self.plan();original['images'][0]['exported_instances']=0
        with self.assertRaisesRegex(ValueError,'empty'):reviewed_plan(original,self.edits())
        edits=self.edits();edits[0]['verified_empty']=True
        self.assertTrue(reviewed_plan(original,edits)['images'][0]['verified_empty'])

    def test_changed_file_list_and_nonboolean_flags_rejected(self):
        for field,value in [('file_name','other.png'),('reviewed','true'),('split','invalid'),('arrangement_group','')]:
            edits=self.edits();edits[0][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):reviewed_plan(self.plan(),edits)

    def test_independent_test_group_and_whole_group_reassignment(self):
        edits=self.edits();edits[3].update(reviewed=True,arrangement_group='layout_c',split='test')
        result=reviewed_plan(self.plan(),edits)
        self.assertEqual(result['target_split'],{'train':2,'val':1,'test':1})
        for row in edits[:2]:row['split']='test'
        edits[3]['split']='train'
        # Two training images are still required.
        with self.assertRaisesRegex(ValueError,'at least two'):reviewed_plan(self.plan(),edits)
        edits=self.edits();edits[3].update(reviewed=True,arrangement_group='layout_a',split='test')
        with self.assertRaisesRegex(ValueError,'multiple splits'):reviewed_plan(self.plan(),edits)

    def test_no_inferred_review_from_annotation_count(self):
        with self.assertRaisesRegex(ValueError,'at least two'):reviewed_plan(self.plan(),self.plan()['images'])

    def test_load_refreshes_counts_and_checks_source_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'export.zip';plan_path=root/'review.json'
            image=io.BytesIO();Image.new('RGB',(30,20),'white').save(image,format='PNG')
            with zipfile.ZipFile(source,'w') as archive:
                archive.writestr('data.yaml',json.dumps({'names':NAMES}))
                for i in range(4):
                    archive.writestr(f'images/train/{i}.png',image.getvalue())
                    archive.writestr(f'labels/train/{i}.txt','0 .1 .1 .9 .1 .9 .9 .1 .9\n')
            plan=self.plan();plan['source_zip_sha256']=digest(source);plan['images'][0]['exported_instances']=777
            plan_path.write_text(json.dumps(plan))
            result,_,_,_=load_plan(source,plan_path)
            self.assertEqual(result['images'][0]['exported_instances'],1)
            self.assertFalse(result['images'][0]['reviewed'])
            plan['source_zip_sha256']='wrong';plan_path.write_text(json.dumps(plan))
            with self.assertRaisesRegex(ValueError,'ZIP differs'):load_plan(source,plan_path)


if __name__=='__main__':unittest.main()
