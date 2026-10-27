"""Copy a YOLO segmentation export, removing only exact duplicate class/polygon rows."""
import argparse
import json
from pathlib import Path
import zipfile
from prepare_yolo_pilot import NAMES, digest, read_export, validate_labels


def clean(source, output):
    if output.exists():
        raise ValueError('Output exists; preserve it and choose a new filename')
    images, labels, names = read_export(source)
    cleaned = {}; removed = {}
    for name in images:
        rows = []
        duplicates = 0
        for line in labels.get(Path(name).stem, '').splitlines():
            if not line.strip():
                continue
            text, _ = validate_labels(line, names)
            if text in rows:
                duplicates += 1
            else:
                rows.append(text)
        cleaned[name] = ''.join(rows)
        validate_labels(cleaned[name], NAMES)
        if duplicates:
            removed[name] = duplicates
    report = {'source_zip': str(source.resolve()), 'source_sha256': digest(source),
              'images': len(images), 'exact_duplicate_rows_removed': removed,
              'other_masks_modified': False}
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_STORED) as archive:
        archive.writestr('data.yaml', json.dumps({'names': NAMES}))
        archive.writestr('cleaning_manifest.json', json.dumps(report, indent=2))
        for name, data in images.items():
            archive.writestr('images/train/' + name, data)
            archive.writestr('labels/train/' + Path(name).stem + '.txt', cleaned[name])
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--zip', type=Path, required=True)
    p.add_argument('--output-zip', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(clean(a.zip, a.output_zip), indent=2))
