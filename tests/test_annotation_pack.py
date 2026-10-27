import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


class AnnotationPackTests(unittest.TestCase):
    def test_limited_selection_covers_entire_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';source.mkdir()
            for i in range(5):
                cv2.imwrite(str(source/f'scene_{i:04}.png'),np.full((120,160,3),i*40,np.uint8))
            config=root/'roi.toml'
            config.write_text('[classical]\npickup_polygon_normalized=[[0.1,0.1],[0.9,0.1],[0.9,0.9],[0.1,0.9]]\n')
            out=root/'selected'
            result=subprocess.run([sys.executable,str(ROOT/'scripts/prepare_board_annotations.py'),
                '--input-dir',str(source),'--output-dir',str(out),'--roi-config',str(config),
                '--limit','3'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            images=json.loads((out/'manifest.json').read_text())['images']
            self.assertEqual([Path(row['source']).name for row in images],
                             ['scene_0000.png','scene_0002.png','scene_0004.png'])

    def test_extension_preserves_selected_files_and_keeps_new_lighting(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);old=root/'existing';(old/'images').mkdir(parents=True)
            source=root/'oct25';source.mkdir()
            original=old/'images'/'s0_scene_0001.png'
            cv2.imwrite(str(original),np.full((120,160,3),200,np.uint8))
            row={'file_name':original.name,'source':'recording_20261026.png',
                 'arrangement_group':'arrangement_1','split':'train'}
            (old/'manifest.json').write_text(json.dumps({'images':[row]}))
            cv2.imwrite(str(source/'duplicate.png'),np.full((120,160,3),200,np.uint8))
            cv2.imwrite(str(source/'different_light.png'),np.full((120,160,3),160,np.uint8))
            cv2.imwrite(str(source/'red.png'),np.full((120,160,3),(0,0,180),np.uint8))
            config=root/'roi.toml'
            config.write_text('[classical]\npickup_polygon_normalized=[[0.1,0.1],[0.9,0.1],[0.9,0.9],[0.1,0.9]]\n')
            out=root/'combined'
            result=subprocess.run([sys.executable,str(ROOT/'scripts/prepare_board_annotations.py'),
                '--existing-pack',str(old),'--input-dir',str(source),'--source-label','oct25',
                '--output-dir',str(out),'--roi-config',str(config),'--limit','5'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            manifest=json.loads((out/'manifest.json').read_text())
            self.assertEqual(manifest['preserved_count'],1)
            self.assertEqual(manifest['added_count'],2)
            self.assertEqual((out/'images'/original.name).read_bytes(),original.read_bytes())
            self.assertEqual(manifest['images'][0]['split'],'train')
            self.assertEqual(manifest['images'][0]['arrangement_group'],'arrangement_1')
            added={p.name for p in (out/'added_images').glob('*.png')}
            self.assertEqual(added,{'oct25_different_light.png','oct25_red.png'})
            for name in added:
                self.assertEqual((out/'images'/name).read_bytes(),(out/'added_images'/name).read_bytes())
            self.assertEqual(json.loads((old/'manifest.json').read_text()),{'images':[row]})


if __name__=='__main__':unittest.main()
