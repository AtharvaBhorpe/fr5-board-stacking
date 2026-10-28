"""Fixture transitions must never alter real-camera data or double-remove a board."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class DemoWorldTest(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / 'demo_world.py'
        cls = next(n for n in ast.parse(source.read_text()).body
                   if isinstance(n, ast.ClassDef) and n.name == 'World')
        function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'panel')
        scope = {'json': json}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
        self.panel = scope['panel']
        self.world = SimpleNamespace(state={}, selected=0, items=['first', 'second'],
                                     completed=set(), frozen=False, write=Mock())

    def send(self, hardware='SIMULATION', phase='completed', task='one'):
        self.panel(self.world, SimpleNamespace(data=json.dumps(
            dict(hardware=hardware, phase=phase, task_id=task))))

    def test_completed_mock_pick_removes_exactly_one_board(self):
        self.send(); self.send()
        self.assertEqual(self.world.items, ['second'])
        self.world.write.assert_called_once()

    def test_real_hardware_is_refused_before_modification(self):
        with self.assertRaises(RuntimeError):
            self.send(hardware='REAL FR5')
        self.assertEqual(self.world.items, ['first', 'second'])
        self.world.write.assert_not_called()

    def test_failed_or_cancelled_pick_does_not_remove(self):
        for phase in ('failed', 'stopped', 'running'):
            self.send(phase=phase)
        self.world.write.assert_not_called()

    def test_freeze_leaves_image_unchanged(self):
        self.world.frozen = True
        self.send()
        self.world.write.assert_not_called()

    def test_unknown_target_is_not_removed(self):
        self.world.selected = None
        self.send()
        self.world.write.assert_not_called()
