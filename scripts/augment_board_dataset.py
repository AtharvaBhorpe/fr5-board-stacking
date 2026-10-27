"""Add four label-preserving appearance variants to training only; keep val/test original."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml
from prepare_yolo_pilot import NAMES, validate_labels


VARIANTS = ('dim', 'bright', 'shadow', 'surface_marks')


def augment(image, text, kind, seed):
    rng = np.random.default_rng(seed)
    h, w = image.shape[:2]
    result = image.astype(np.float32)
    if kind == 'dim':
        result *= rng.uniform(.65, .82)
    elif kind == 'bright':
        # Compress highlights gently instead of clipping white board edges.
        result = 255 * np.power(result / 255, rng.uniform(.78, .9))
    elif kind == 'shadow':
        y, x = np.ogrid[:h, :w]
        angle = rng.uniform(-np.pi, np.pi)
        distance = ((x / w - rng.uniform(.3, .7)) * np.cos(angle)
                    + (y / h - rng.uniform(.3, .7)) * np.sin(angle))
        soft_edge = .5 + .5 * np.tanh(distance / rng.uniform(.04, .12))
        result *= (1 - rng.uniform(.2, .4) * soft_edge)[..., None]
    elif kind == 'surface_marks':
        mask = np.zeros((h, w), np.uint8)
        hands = np.zeros_like(mask)
        for line in text.splitlines():
            values = line.split()
            points = np.array(list(map(float, values[1:])), np.float32).reshape(-1, 2)
            points = np.rint(points * [w, h]).astype(np.int32)
            cv2.fillPoly(hands if int(values[0]) == 3 else mask, [points], 255)
        mask[hands > 0] = 0
        # Keep small neutral marks inside visible surfaces, away from object edges.
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8))
        ys, xs = np.nonzero(mask)
        marks = np.zeros_like(mask)
        if len(xs):
            for index in rng.choice(len(xs), size=min(16, len(xs)), replace=False):
                radius = int(rng.integers(1, max(3, min(h, w) // 160)))
                cv2.circle(marks, (int(xs[index]), int(ys[index])), radius, 255, -1)
            marks = cv2.GaussianBlur(marks, (5, 5), 0)
            alpha = (marks.astype(np.float32) / 255 * rng.uniform(.3, .55))
            alpha[mask == 0] = 0
            result *= (1 - alpha)[..., None]
    else:
        raise ValueError('Unknown variant: ' + kind)
    return np.clip(np.rint(result), 0, 255).astype(np.uint8)


def build_augmented(dataset, output):
    if output.exists():
        raise ValueError('Output exists; choose a new folder to preserve previous work')
    config = yaml.safe_load((dataset / 'data.yaml').read_text())
    manifest = json.loads((dataset / 'manifest.json').read_text())
    names = config['names']
    if isinstance(names, dict):
        names = [names[i] if i in names else names[str(i)] for i in range(len(names))]
    if names != NAMES:
        raise ValueError('Build the canonical reviewed dataset first')
    if manifest.get('appearance_augmentation'):
        raise ValueError('Input already augmented; use the reviewed originals')
    records = []
    seen = set()
    for split in ('train', 'val', 'test'):
        for source in sorted((dataset / 'images' / split).glob('*')):
            if not source.is_file():
                continue
            label = dataset / 'labels' / split / (source.stem + '.txt')
            text = label.read_text()
            validate_labels(text, NAMES)
            image = cv2.imread(str(source))
            if image is None:
                raise ValueError('Unreadable image: ' + str(source))
            fingerprint = hashlib.sha256(source.read_bytes()).hexdigest()
            if fingerprint in seen:
                raise ValueError('Duplicate original image; check the reviewed splits')
            seen.add(fingerprint)
            records.append((source, label, split, fingerprint))
    actual = dict(Counter(r[2] for r in records))
    if actual != manifest['images_per_split']:
        raise ValueError('Dataset counts disagree with reviewed manifest')
    output.mkdir(parents=True)
    lineage = []
    preview_rows = []
    for source, label, split, fingerprint in records:
        images = output / 'images' / split
        labels = output / 'labels' / split
        images.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, images / source.name)
        shutil.copyfile(label, labels / label.name)
        if split != 'train':
            continue
        image = cv2.imread(str(source))
        text = label.read_text()
        previews = [image]
        for kind in VARIANTS:
            seed = int(hashlib.sha256((fingerprint + kind).encode()).hexdigest()[:8], 16)
            name = f'{source.stem}__appearance_{kind}.png'
            if (images / name).exists() or (labels / Path(name).with_suffix('.txt')).exists():
                raise ValueError('Augmented filename collision')
            result = augment(image, text, kind, seed)
            if not cv2.imwrite(str(images / name), result):
                raise OSError('Cannot save augmented image')
            shutil.copyfile(label, labels / Path(name).with_suffix('.txt'))
            lineage.append({'file_name': name, 'source': source.name, 'source_sha256': fingerprint,
                            'split': 'train', 'variant': kind, 'seed': seed,
                            'labels_unchanged': True})
            previews.append(result)
        if len(preview_rows) < 8:
            tiles = []
            for title, frame in zip(('original', *VARIANTS), previews):
                tile = cv2.resize(frame, (320, 180))
                cv2.putText(tile, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
                tiles.append(tile)
            preview_rows.append(np.hstack(tiles))
    config['path'] = str(output.resolve())
    (output / 'data.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    manifest['appearance_augmentation'] = {'source_dataset': str(dataset.resolve()),
        'variants': list(VARIANTS), 'lineage': lineage, 'val_test_changed': False,
        'geometry_changed': False, 'manual_appearance_review_completed': False}
    manifest['images_per_split'] = {s: n * (5 if s == 'train' else 1) for s, n in actual.items()}
    for category, count in manifest.get('instances_per_split', {}).get('train', {}).items():
        manifest['instances_per_split']['train'][category] = count * 5
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    if preview_rows:
        cv2.imwrite(str(output / 'augmentation_preview.jpg'), np.vstack(preview_rows))
    return {'originals_per_split': actual, 'images_per_split': manifest['images_per_split'],
            'generated_train_images': len(lineage), 'labels_reannotation_needed': False,
            'data_yaml': str((output / 'data.yaml').resolve())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(build_augmented(a.dataset, a.output_dir), indent=2))


if __name__ == '__main__':
    main()
