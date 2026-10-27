"""Create a portable training-only archive; no ROS, credentials, checkpoints or raw bags."""
import argparse
from collections import Counter
import hashlib
import io
import json
from pathlib import Path
import tarfile
import yaml
from prepare_yolo_pilot import NAMES, validate_labels


def package(dataset, output, repo):
    if output.exists():
        raise ValueError('Archive exists; preserve it and choose a new filename')
    config = yaml.safe_load((dataset / 'data.yaml').read_text())
    names = config['names']
    if isinstance(names, dict):
        names = [names[i] if i in names else names[str(i)] for i in range(len(names))]
    if names != NAMES:
        raise ValueError('Expected canonical VARa, VARb, VARc, hand dataset')
    manifest = json.loads((dataset / 'manifest.json').read_text())
    files = []
    counts = Counter()
    for split in ('train', 'val', 'test'):
        for image in sorted((dataset / 'images' / split).glob('*')):
            if not image.is_file():
                raise ValueError('Unexpected dataset directory')
            label = dataset / 'labels' / split / (image.stem + '.txt')
            validate_labels(label.read_text(), names)
            files.extend([(image, 'dataset/images/' + split + '/' + image.name),
                          (label, 'dataset/labels/' + split + '/' + label.name)])
            counts[split] += 1
    if dict(counts) != manifest['images_per_split']:
        raise ValueError('Dataset counts differ from reviewed manifest')
    files.extend([(dataset / 'manifest.json', 'dataset/manifest.json'),
        (repo / 'config/brev_training/pixi.toml', 'pixi.toml'),
        (repo / 'scripts/train_yolo_pilot.py', 'scripts/train_yolo_pilot.py'),
        (repo / 'scripts/prepare_yolo_pilot.py', 'scripts/prepare_yolo_pilot.py')])
    config['path'] = '.'
    contents = {'dataset/data.yaml': yaml.safe_dump(config, sort_keys=False).encode()}
    checksums = {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output, 'x') as archive:
        for path, name in files:
            checksums[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            archive.add(path, arcname='fr5_yolo_training_001/' + name, recursive=False)
        contents['SHA256SUMS'] = ''.join(f'{value}  {name}\n' for name, value in sorted(checksums.items())).encode()
        for name, data in contents.items():
            info = tarfile.TarInfo('fr5_yolo_training_001/' + name)
            info.size = len(data); info.mode = 0o644
            archive.addfile(info, io.BytesIO(data))
    checksum = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(output.suffix + '.sha256').write_text(checksum + '  ' + output.name + '\n')
    return {'archive': str(output.resolve()), 'bytes': output.stat().st_size,
            'images_per_split': dict(counts), 'sha256': checksum,
            'remote_step': 'Verify SHA256SUMS, then set dataset/data.yaml path to the absolute remote dataset directory'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(package(a.dataset, a.output, Path(__file__).resolve().parents[1]), indent=2))
