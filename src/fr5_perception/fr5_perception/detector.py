"""Stage A image observations; no metric poses until calibration is validated."""

from abc import ABC, abstractmethod
from collections import Counter

import cv2
import numpy as np

from .colour_masks import foreground_mask
from .measurements import describe_region


class Detector(ABC):
    @abstractmethod
    def detect(self, bgr):
        """Return image observations and explicit calibration/execution status."""


def region_mask(shape, points):
    height, width = shape[:2]
    mask = np.full((height, width), 255, np.uint8)
    if not points:
        return mask
    polygon = np.asarray(points, dtype=float)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
            or not np.isfinite(polygon).all() or np.any(polygon < 0) or np.any(polygon > 1)):
        raise ValueError('Region vertices must be normalized finite [x, y] pairs in [0, 1].')
    pixels = np.rint(polygon * [width - 1, height - 1]).astype(np.int32)
    if not cv2.isContourConvex(pixels) or cv2.contourArea(pixels) <= 0:
        raise ValueError('Region must be a nondegenerate convex polygon with vertices in perimeter order.')
    mask[:] = 0
    cv2.fillPoly(mask, [pixels], 255)
    return mask


class ClassicalDetector(Detector):
    def __init__(self, config):
        self.config = config
        self.settings = config['classical']
        settings = self.settings
        if not (0 < settings['min_area_fraction'] < settings['max_area_fraction'] <= 1):
            raise ValueError('Invalid candidate area limits.')
        if not (0 < settings['min_rectangularity'] <= 1 and settings['max_aspect_ratio'] >= 1):
            raise ValueError('Invalid rectangle filters.')
        if not (0 < settings['contour_epsilon_fraction'] < 0.1):
            raise ValueError('Invalid contour approximation setting.')
        if not (0 <= settings['min_value'] <= 255 and 0 <= settings['max_saturation'] <= 255):
            raise ValueError('HSV thresholds must be within [0, 255].')
        self.marker_detectors = []
        for name in config['markers']['dictionaries']:
            if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
                raise ValueError(f'Unknown marker dictionary: {name}')
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
            parameters = cv2.aruco.DetectorParameters()
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.marker_detectors.append((name, cv2.aruco.ArucoDetector(dictionary, parameters)))

    def detect_markers(self, bgr):
        observations = []
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        for name, detector in self.marker_detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                for polygon, marker_id in zip(corners, ids.flatten()):
                    observations.append({'dictionary_used': name, 'id': int(marker_id),
                                         'pixel_corners': polygon.reshape(4, 2).tolist()})
        return observations

    def foreground(self, bgr):
        ranges = self.settings.get('foreground_hsv_ranges', [
            [0, 0, self.settings['min_value'],
             179, self.settings['max_saturation'], 255]])
        return foreground_mask(bgr, ranges)

    def detect(self, bgr):
        if not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('Expected a uint8 BGR image with three channels.')
        if min(bgr.shape[:2]) < 16:
            raise ValueError('Image is too small for detection.')
        settings = self.settings
        roi = region_mask(bgr.shape, settings['pickup_polygon_normalized'])
        foreground = cv2.bitwise_and(self.foreground(bgr), roi)
        boundary = cv2.subtract(roi, cv2.erode(roi, np.ones((5, 5), np.uint8),
                                             borderType=cv2.BORDER_CONSTANT, borderValue=0))
        markers = self.detect_markers(bgr)
        counts = Counter((marker['dictionary_used'], marker['id']) for marker in markers)
        duplicates = [{'dictionary_used': name, 'id': marker_id, 'count': count}
                      for (name, marker_id), count in counts.items() if count > 1]
        contours, _ = cv2.findContours(foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        image_area = bgr.shape[0] * bgr.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if not settings['min_area_fraction'] <= area / image_area <= settings['max_area_fraction']:
                continue
            polygon = cv2.approxPolyDP(contour, settings['contour_epsilon_fraction'] * cv2.arcLength(contour, True), True)
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            component = np.zeros_like(roi)
            cv2.drawContours(component, [contour], -1, 255, cv2.FILLED)
            if np.any(cv2.bitwise_and(component, boundary)):
                continue
            _, (side_a, side_b), _ = cv2.minAreaRect(contour)
            if min(side_a, side_b) <= 0:
                continue
            rectangularity = area / (side_a * side_b)
            if (rectangularity < settings['min_rectangularity']
                    or max(side_a, side_b) / min(side_a, side_b) > settings['max_aspect_ratio']):
                continue
            corners = polygon.reshape(4, 2)
            contained = []
            for marker in markers:
                center = np.mean(marker['pixel_corners'], axis=0)
                if cv2.pointPolygonTest(polygon, tuple(float(v) for v in center), False) >= 0:
                    contained.append({'dictionary_used': marker['dictionary_used'], 'id': marker['id']})
            candidates.append({'pixel_corners': corners.tolist(),
                               'pixel_center': np.mean(corners, axis=0).tolist(),
                               'area_pixels': float(area), 'rectangularity': float(rectangularity),
                               'contained_markers': contained, 'metric_pose': None,
                               **describe_region(bgr, component)})
        candidates.sort(key=lambda candidate: candidate['pixel_center'][0])
        for index, candidate in enumerate(candidates):
            candidate['id'] = f'candidate_{index + 1}'
        warnings = ['Metric camera/table calibration and surface height are not validated.']
        if not settings['pickup_polygon_normalized']:
            warnings.append('Pickup region is unassigned; candidates are diagnostic image regions only.')
        if duplicates:
            warnings.append('Repeated marker IDs cannot be used as unique scene anchors.')
        return {'image_size': [bgr.shape[1], bgr.shape[0]], 'mode': 'pixel_preview',
                'execution_allowed': False, 'calibration_valid': False,
                'quad_candidates': candidates, 'markers': markers,
                'duplicate_marker_ids': duplicates, 'warnings': warnings}
