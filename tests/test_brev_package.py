import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from package_brev_training import package
from prepare_yolo_pilot import NAMES


class BrevPackageTests(unittest.TestCase):
    def test_portable_paths_counts_checksum_no_extra_files_and_original_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); data = root / 'data'
            for split in ('train', 'val', 'test'):
                (data / 'images' / split).mkdir(parents=True)
                (data / 'labels' / split).mkdir(parents=True)
                (data / 'images' / split / 'a.png').write_bytes(b'test-image')
                (data / 'labels' / split / 'a.txt').write_text('0 .1 .1 .9 .1 .9 .9 .1 .9\n')
            original = yaml.safe_dump({'path': '/old/laptop/path', 'names': NAMES,
                'train': 'images/train', 'val': 'images/val', 'test': 'images/test'})
            (data / 'data.yaml').write_text(original)
            (data / 'manifest.json').write_text(json.dumps({'images_per_split': {'train': 1, 'val': 1, 'test': 1}}))
            (data / 'secret.env').write_text('must not be uploaded')
            repo = Path(__file__).resolve().parents[1]
            output = root / 'cloud.tar'
            result = package(data, output, repo)
            self.assertEqual(result['images_per_split'], {'train': 1, 'val': 1, 'test': 1})
            self.assertEqual((data / 'data.yaml').read_text(), original)
            with tarfile.open(output) as archive:
                self.assertFalse(any('secret' in p for p in archive.getnames()))
                prefix = 'fr5_yolo_training_001/'
                config = yaml.safe_load(archive.extractfile(prefix + 'dataset/data.yaml').read())
                self.assertEqual(config['path'], '.')
                for line in archive.extractfile(prefix + 'SHA256SUMS').read().decode().splitlines():
                    digest, name = line.split('  ', 1)
                    self.assertEqual(hashlib.sha256(archive.extractfile(prefix + name).read()).hexdigest(), digest)
            with self.assertRaisesRegex(ValueError, 'Archive exists'):
                package(data, output, repo)
