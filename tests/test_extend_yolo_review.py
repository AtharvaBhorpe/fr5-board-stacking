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
from prepare_yolo_pilot import NAMES,digest,read_export,build,validate_frozen_reviews
from extend_yolo_review import extend
from review_yolo_plan import load_plan,reviewed_plan


class ExtendTests(unittest.TestCase):
    def setup_files(self,root):
        old=root/'old.zip';new=root/'new.zip';plan_path=root/'old.json'
        polygon='0 .1 .1 .9 .1 .9 .9 .1 .9\n';images=[]
        for i in range(4):
            b=io.BytesIO();Image.new('RGB',(64,48),(i*40,20,70)).save(b,format='PNG');images.append(b.getvalue())
        with zipfile.ZipFile(old,'w') as z:
            z.writestr('data.yaml',json.dumps({'names':NAMES}))
            for i in range(3):z.writestr(f'images/train/{i}.png',images[i]);z.writestr(f'labels/train/{i}.txt',polygon)
        plan={'source_zip_sha256':digest(old),'images':[{'file_name':f'{i}.png','exported_instances':1,
            'reviewed':True,'arrangement_group':f'old_group_{i}','split':['train','val','test'][i],
            'verified_empty':False} for i in range(3)]}
        plan_path.write_text(json.dumps(plan))
        with zipfile.ZipFile(new,'w') as z:
            z.writestr('data.yaml',json.dumps({'names':NAMES}))
            z.writestr('images/train/0.png',images[0]);z.writestr('labels/train/0.txt','')
            z.writestr('images/train/added.png',images[3]);z.writestr('labels/train/added.txt',polygon)
        return old,new,plan_path,plan,images

    def test_old_assignments_and_labels_preserved_new_rows_only_pending(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);old,new,plan_path,original,images=self.setup_files(root)
            old_bytes=plan_path.read_bytes();result=extend(old,plan_path,[new],root/'combined.zip',root/'combined.json')
            self.assertEqual(result['new_images_to_review'],1);self.assertEqual(result['exact_duplicate_images_skipped'],1)
            plan=json.loads((root/'combined.json').read_text())
            for before,after in zip(original['images'],plan['images']):
                self.assertEqual(before,{k:after[k] for k in before});self.assertTrue(after['locked'])
            self.assertFalse(plan['images'][-1]['reviewed']);self.assertFalse(plan['images'][-1]['locked'])
            self.assertEqual(plan_path.read_bytes(),old_bytes)
            _,labels,_=read_export(root/'combined.zip');self.assertTrue(labels['0'])
            loaded,_,_,_=load_plan(root/'combined.zip',root/'combined.json')
            validate_frozen_reviews(loaded)

    def test_old_review_cannot_be_changed_in_browser_or_builder(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);old,new,plan_path,_,_=self.setup_files(root)
            extend(old,plan_path,[new],root/'combined.zip',root/'combined.json')
            original=json.loads((root/'combined.json').read_text());bad=copy.deepcopy(original)
            bad['images'][2]['split']='train'
            with self.assertRaisesRegex(ValueError,'locked'):validate_frozen_reviews(bad)
            with self.assertRaisesRegex(ValueError,'locked'):build(root/'combined.zip',bad,root/'out')
            # Give the original plan a second training row via the new image, then attempt editing the old test row.
            edits=copy.deepcopy(original['images']);edits[-1].update(reviewed=True,arrangement_group='new_group',split='train')
            edits[2]['split']='train'
            with self.assertRaisesRegex(ValueError,'locked'):reviewed_plan(original,edits)
            edits[2]['split']='test';self.assertEqual(reviewed_plan(original,edits)['target_split'],{'train':2,'val':1,'test':1})

    def test_changed_existing_annotations_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);old,new,plan_path,_,images=self.setup_files(root)
            with zipfile.ZipFile(root/'changed.zip','w') as z:
                z.writestr('data.yaml',json.dumps({'names':NAMES}));z.writestr('images/train/0.png',images[0])
                z.writestr('labels/train/0.txt','2 .1 .1 .9 .1 .9 .9 .1 .9\n')
            with self.assertRaisesRegex(ValueError,'changed annotations'):extend(old,plan_path,[root/'changed.zip'],root/'out.zip',root/'out.json')
            self.assertFalse((root/'out.zip').exists())

    def test_same_image_under_new_name_is_not_a_new_training_sample(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);old,new,plan_path,_,images=self.setup_files(root)
            with zipfile.ZipFile(root/'duplicate.zip','w') as z:
                z.writestr('data.yaml',json.dumps({'names':NAMES}));z.writestr('images/train/renamed.png',images[0])
                z.writestr('labels/train/renamed.txt','0 .1 .1 .9 .1 .9 .9 .1 .9\n')
            result=extend(old,plan_path,[new,root/'duplicate.zip'],root/'out.zip',root/'out.json')
            self.assertEqual(result['new_images_to_review'],1);self.assertEqual(result['exact_duplicate_images_skipped'],2)


if __name__=='__main__':unittest.main()
