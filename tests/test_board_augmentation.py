import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from augment_board_dataset import augment, build_augmented, VARIANTS
from prepare_yolo_pilot import NAMES


class AugmentationTests(unittest.TestCase):
    def test_marks_stay_inside_board_and_outside_hand(self):
        image = np.full((128, 192, 3), 180, np.uint8)
        text = '0 .2 .2 .8 .2 .8 .8 .2 .8\n3 .4 .4 .6 .4 .6 .6 .4 .6\n'
        result = augment(image, text, 'surface_marks', 42)
        changed = np.any(result != image, axis=2)
        self.assertTrue(changed.any())
        self.assertFalse(changed[:26].any())
        self.assertFalse(changed[:, :39].any())
        self.assertFalse(changed[52:76, 78:114].any())
        for kind in VARIANTS:
            a = augment(image, text, kind, 42)
            self.assertEqual(a.shape, image.shape)
            self.assertEqual(a.dtype, np.uint8)
            np.testing.assert_array_equal(a, augment(image, text, kind, 42))

    def test_only_train_augmented_labels_and_originals_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); source = root / 'source'; output = root / 'aug'
            counts = {'train': 2, 'val': 1, 'test': 1}
            text = '0 .2 .2 .8 .2 .8 .8 .2 .8\n'
            for split, count in counts.items():
                (source / 'images' / split).mkdir(parents=True)
                (source / 'labels' / split).mkdir(parents=True)
                for i in range(count):
                    cv2.imwrite(str(source / 'images' / split / f'{split}{i}.png'),
                                np.full((64, 96, 3), 60 + 30 * len(list(source.rglob('*.png'))), np.uint8))
                    (source / 'labels' / split / f'{split}{i}.txt').write_text(text)
            (source / 'data.yaml').write_text(yaml.safe_dump({'path': str(source), 'names': NAMES,
                'train': 'images/train', 'val': 'images/val', 'test': 'images/test'}))
            (source / 'manifest.json').write_text(json.dumps({'images_per_split': counts}))
            before = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in source.rglob('*') if p.is_file()}
            report = build_augmented(source, output)
            self.assertEqual(report['images_per_split'], {'train': 10, 'val': 1, 'test': 1})
            for name, fingerprint in before.items():
                self.assertEqual(hashlib.sha256((source / name).read_bytes()).hexdigest(), fingerprint)
                if name.startswith(('images/', 'labels/')):
                    self.assertEqual((output / name).read_bytes(), (source / name).read_bytes())
            for path in (output / 'labels/train').glob('*'):
                self.assertEqual(path.read_text(), text)
            self.assertEqual(len(list((output / 'images/val').glob('*'))), 1)
            self.assertEqual(len(list((output / 'images/test').glob('*'))), 1)
            with self.assertRaisesRegex(ValueError, 'Output exists'):
                build_augmented(source, output)
            with self.assertRaisesRegex(ValueError, 'already augmented'):
                build_augmented(output, root / 'again')


if __name__ == '__main__':
    unittest.main()
