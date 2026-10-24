"""Inspect a saved camera image without connecting to a robot."""

import argparse
import json
from pathlib import Path
import tomllib

import cv2
import numpy as np

from .detector import ClassicalDetector, region_mask


def overlay(image, report, config):
    result = image.copy()
    height, width = image.shape[:2]
    # Display coordinates only; this grid never defines a metric table frame.
    for column in range(6):
        x = round(column * width / 6)
        cv2.line(result, (x, 0), (x, height - 1), (150, 100, 20), 1)
        for row in range(4):
            y = round(row * height / 4)
            cv2.putText(result, f'{chr(65 + column)}{row + 1}', (x + 10, y + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)
    for row in range(1, 4):
        y = round(row * height / 4)
        cv2.line(result, (0, y), (width - 1, y), (150, 100, 20), 1)
    points = config['classical']['pickup_polygon_normalized']
    if points:
        polygon = np.rint(np.asarray(points) * [width - 1, height - 1]).astype(np.int32)
        cv2.polylines(result, [polygon], True, (0, 200, 255), 3)
    for marker in report['markers']:
        polygon = np.rint(marker['pixel_corners']).astype(np.int32)
        cv2.polylines(result, [polygon], True, (255, 200, 0), 2)
        cv2.putText(result, str(marker['id']), tuple(polygon[0]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 150, 0), 2)
    for candidate in report['quad_candidates']:
        polygon = np.asarray(candidate['pixel_corners'], np.int32)
        cv2.polylines(result, [polygon], True, (30, 255, 30), 2)
        cv2.putText(result, candidate['id'], tuple(polygon[0]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (30, 255, 30), 2)
    cv2.putText(result, 'PIXEL PREVIEW - NO ROBOT GRASP OUTPUT', (20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--config', default='config/perception.toml')
    args = parser.parse_args()
    try:
        with open(args.config, 'rb') as source:
            config = tomllib.load(source)
        image = cv2.imread(args.image)
        if image is None:
            raise ValueError('Cannot read input image.')
        detector = ClassicalDetector(config)
        report = detector.detect(image)
        folder = Path(args.output_dir)
        folder.mkdir(parents=True, exist_ok=True)
        for name, data in (('overlay.png', overlay(image, report, config)),
                           ('foreground.png', cv2.bitwise_and(detector.foreground(image),
                            region_mask(image.shape, config['classical']['pickup_polygon_normalized'])))):
            if not cv2.imwrite(str(folder / name), data):
                raise OSError(f'Cannot write {name}.')
        (folder / 'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({'quad_candidates': len(report['quad_candidates']),
                          'markers': len(report['markers']), 'execution_allowed': False,
                          'warnings': report['warnings'], 'report': str(folder / 'report.json')}, indent=2))
        return 0
    except (OSError, ValueError, KeyError) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}))
        return 2
