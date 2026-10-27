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
from prepare_yolo_pilot import NAMES,build,digest,validate_labels,read_export,merge_exports,canonical_names


class PilotTests(unittest.TestCase):
    def test_new_labels_and_legacy_aliases_preserve_class_order(self):
        self.assertEqual(canonical_names(['large_white','small_white','small_red','hand']),NAMES)
        self.assertEqual(canonical_names({'0':'VARa','1':'VARb','2':'VARc','3':'hand'}),NAMES)
        text,counts=validate_labels('0 .1 .1 .9 .1 .9 .9 .1 .9',
            ['small_red','hand','large_white','small_white'])
        self.assertTrue(text.startswith('2 '));self.assertEqual(counts,{'VARc':1})
        for names in (['VARa','large_white','VARc','hand'],['VARa','VARb','wrong','hand']):
            with self.subTest(names=names),self.assertRaises(ValueError):canonical_names(names)

    def test_merging_legacy_and_new_exports_uses_canonical_labels(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,_=self.export(root);legacy=root/'legacy.zip'
            with zipfile.ZipFile(source) as original,zipfile.ZipFile(legacy,'w') as archive:
                archive.writestr('data.yaml',json.dumps({'names':['large_white','small_white','small_red','hand']}))
                archive.writestr('images/train/extra.png',original.read('images/train/scene_3.png'))
                archive.writestr('labels/train/extra.txt','0 .1 .1 .9 .1 .9 .9 .1 .9\n')
            merge_exports([source,legacy],root/'combined.zip')
            _,labels,names=read_export(root/'combined.zip')
            self.assertEqual(names,NAMES);self.assertTrue(labels['extra'].startswith('0 '))

    def test_merge_deduplicates_and_keeps_nonempty_labels(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,_=self.export(root);second=root/'second.zip'
            with zipfile.ZipFile(source) as original,zipfile.ZipFile(second,'w') as z:
                z.writestr('data.yaml',original.read('data.yaml'))
                z.writestr('images/train/scene_3.png',original.read('images/train/scene_3.png'))
                z.writestr('labels/train/scene_3.txt','0 .1 .1 .9 .1 .9 .9 .1 .9\n')
            result=merge_exports([source,second],root/'merged.zip')
            self.assertEqual(result['images'],4);self.assertEqual(result['annotated_images'],4)
            _,labels,names=read_export(root/'merged.zip')
            self.assertTrue(labels['scene_3'].startswith('3 '));self.assertEqual(names,NAMES)

    def test_cvat_xml_export_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'xml.zip'
            with zipfile.ZipFile(path,'w') as z:z.writestr('annotations.xml','<annotations/>')
            with self.assertRaisesRegex(ValueError,'CVAT XML export'):read_export(path)

    def export(self,folder):
        path=folder/'cvat.zip'
        with zipfile.ZipFile(path,'w') as z:
            z.writestr('data.yaml',json.dumps({'names':list(reversed(NAMES))}))
            for i in range(4):
                b=io.BytesIO();Image.new('RGB',(64,48),(i*30,20,40)).save(b,format='PNG')
                z.writestr(f'images/train/scene_{i}.png',b.getvalue())
                if i<3:z.writestr(f'labels/train/scene_{i}.txt','0 .1 .1 .9 .1 .9 .9 .1 .9\n')
        plan={'source_zip_sha256':digest(path),'images':[
            {'file_name':f'scene_{i}.png','reviewed':i<3,'arrangement_group':f'group_{i}',
             'split':'train' if i<2 else 'val','verified_empty':False} for i in range(4)]}
        return path,plan

    def test_excludes_unreviewed_and_remaps_classes(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,plan=self.export(root)
            report=build(source,plan,root/'out')
            self.assertEqual(report['images_per_split'],{'train':2,'val':1})
            self.assertFalse((root/'out/images/val/scene_3.png').exists())
            self.assertTrue((root/'out/labels/train/scene_0.txt').read_text().startswith('3 '))

    def test_empty_frame_requires_explicit_review(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,plan=self.export(root);plan['images'][3]['reviewed']=True
            with self.assertRaisesRegex(ValueError,'verified_empty'):build(source,plan,root/'out')
            plan['images'][3]['verified_empty']=True
            self.assertEqual(build(source,plan,root/'out')['images_per_split']['val'],2)

    def test_arrangement_leakage_and_changed_export_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,plan=self.export(root)
            bad=copy.deepcopy(plan);bad['images'][2]['arrangement_group']='group_0'
            with self.assertRaisesRegex(ValueError,'multiple splits'):build(source,bad,root/'out')
            bad=copy.deepcopy(plan);bad['source_zip_sha256']='wrong'
            with self.assertRaisesRegex(ValueError,'Export changed'):build(source,bad,root/'out')

    def test_requested_split_count_must_be_completed(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,plan=self.export(root)
            plan['target_split']={'train':27,'val':9}
            with self.assertRaisesRegex(ValueError,'targets'):build(source,plan,root/'out')

    def test_test_set_is_built_but_not_used_as_validation(self):
        import yaml
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source,plan=self.export(root)
            plan['images'][3].update(reviewed=True,verified_empty=True,split='test')
            plan['target_split']={'train':2,'val':1,'test':1}
            report=build(source,plan,root/'out')
            self.assertEqual(report['images_per_split'],plan['target_split'])
            config=yaml.safe_load((root/'out/data.yaml').read_text())
            self.assertEqual(config['test'],'images/test');self.assertEqual(config['val'],'images/val')
            self.assertTrue((root/'out/images/test/scene_3.png').is_file())
            self.assertFalse((root/'out/images/train/scene_3.png').exists())
            self.assertFalse((root/'out/images/val/scene_3.png').exists())
            bad=copy.deepcopy(plan);bad['images'][3]['arrangement_group']='group_0'
            with self.assertRaisesRegex(ValueError,'multiple splits'):build(source,bad,root/'bad')

    def test_boxes_nan_degenerate_and_duplicate_polygons_rejected(self):
        polygon='0 .1 .1 .9 .1 .9 .9 .1 .9'
        for row in ('0 .5 .5 .2 .2','0 nan .1 .9 .1 .9 .9','0 .1 .1 .2 .2 .3 .3',polygon+'\n'+polygon):
            with self.subTest(row=row),self.assertRaises(ValueError):validate_labels(row,NAMES)


if __name__=='__main__':unittest.main()
