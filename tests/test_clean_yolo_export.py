import io
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from clean_yolo_export import clean
from prepare_yolo_pilot import read_export


class CleanTests(unittest.TestCase):
    def test_only_same_class_same_polygon_is_removed_and_original_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); source = root / 'input.zip'; output = root / 'clean.zip'
            data = io.BytesIO(); Image.new('RGB', (20, 20), 'white').save(data, format='PNG')
            row = '.1 .1 .9 .1 .9 .9 .1 .9\n'
            with zipfile.ZipFile(source, 'w') as z:
                z.writestr('data.yaml', 'names: [large_white, small_white, small_red, hand]')
                z.writestr('images/train/a.png', data.getvalue())
                z.writestr('labels/train/a.txt', '0 ' + row + '0 ' + row + '1 ' + row)
            before = source.read_bytes()
            report = clean(source, output)
            self.assertEqual(report['exact_duplicate_rows_removed'], {'a.png': 1})
            images, labels, _ = read_export(output)
            self.assertEqual(images['a.png'], data.getvalue())
            self.assertEqual(len(labels['a'].splitlines()), 2)
            self.assertEqual(source.read_bytes(), before)
            with self.assertRaisesRegex(ValueError, 'Output exists'):
                clean(source, output)
