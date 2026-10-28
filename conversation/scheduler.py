"""Bounded batch intents and progress, independent of ROS and model output."""
from collections import Counter
from dataclasses import dataclass, field
import math
import re
import uuid

MAX_PICKS = 10
ORDINALS = ('first', 'second', 'third', 'fourth', 'fifth', 'sixth', 'seventh', 'eighth', 'ninth', 'tenth')


def ordinal(number):
    return ORDINALS[number-1] if 1 <= number <= MAX_PICKS else str(number)


NUMBERS = {word: i for i, word in enumerate(
    ('zero', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'ten'))}


def batch_request(text):
    text = re.sub(r'[.!?]+$', '', text.lower().strip())
    text = re.sub(r'^(please |can you |could you )', '', text)
    text = re.sub(r' please$', '', text)
    if text in ('sort all visible boards', 'sort all boards', 'pick all visible boards'):
        return ('all', None)
    m = re.fullmatch(r'pick(?: up| and place)? (\d+|' + '|'.join(NUMBERS) +
                     r') (?:variant\s+|var\s*)([abc])(?: boards?)?', text)
    if m:
        count = NUMBERS.get(m[1], int(m[1]) if m[1].isdigit() else 0)
        return ('count', ('VAR' + m[2], count))
    return None


def visible_counts(scene):
    """Count calibrated source detections; reject overlapping duplicate boxes conservatively."""
    boards = scene.get('boards', [])
    if not isinstance(boards, list):
        raise ValueError('Invalid board detections.')
    counts, positions = Counter(), []
    for b in boards:
        if not isinstance(b, dict):
            raise ValueError('Invalid board detection.')
        if b.get('variant') not in ('VARa', 'VARb', 'VARc') or b.get('zone') != 'pick':
            continue
        if b.get('inside_calibration') is not True or b.get('size_ok') is not True:
            raise ValueError('A source board is outside calibration or has uncertain dimensions.')
        x, y = b.get('x'), b.get('y')
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or not math.isfinite(x+y):
            raise ValueError('Invalid source-board coordinates.')
        if any(math.hypot(x-px, y-py) < .04 for px, py in positions):
            raise ValueError('Overlapping or duplicate detections; inspect the scene.')
        positions.append((x, y))
        counts[b['variant']] += 1
    return counts


def make_plan(request, scene):
    counts = visible_counts(scene)
    if request[0] == 'all':
        items = [v for v in ('VARa', 'VARb', 'VARc') for _ in range(counts[v])]
    else:
        variant, count = request[1]
        if not 1 <= count <= MAX_PICKS:
            raise ValueError(f'Request between 1 and {MAX_PICKS} boards.')
        if counts[variant] < count:
            raise ValueError(f'Only {counts[variant]} freely visible {variant} boards; requested {count}.')
        items = [variant] * count
    if not 1 <= len(items) <= MAX_PICKS:
        raise ValueError(f'A batch must contain 1 to {MAX_PICKS} visible boards.')
    destinations = {d.get('variant') for d in scene.get('destinations', []) if isinstance(d, dict)}
    if any(v not in destinations for v in items):
        raise ValueError('A destination is missing.')
    return Batch(items)


@dataclass
class Batch:
    items: list
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    completed: int = 0
    phase: str = 'checking'
    reason: str = ''
    events: list = field(default_factory=list)

    def snapshot(self):
        return {'id': self.id, 'total': len(self.items), 'completed': self.completed, 'phase': self.phase,
                'remaining': self.items[self.completed:], 'reason': self.reason}

    def finish(self, phase, reason=''):
        self.phase, self.reason = phase, reason
