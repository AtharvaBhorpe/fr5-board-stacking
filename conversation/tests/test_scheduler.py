import ast
import copy
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scheduler import batch_request, make_plan, visible_counts, ordinal
from policy import intent, block_reason, visual_agreement


def board(variant, x):
    return dict(variant=variant, x=x, y=0.0, zone='pick', inside_calibration=True, size_ok=True)


class SchedulerTest(unittest.TestCase):
    def setUp(self):
        self.scene = dict(stamp=100.0, live=True, calibrated=True,
                          boards=[board('VARb', .1), board('VARb', .3)],
                          destinations=[{'variant': 'VARb'}])

    def test_bounded_intents(self):
        self.assertEqual(batch_request('Please pick two variant B boards.'), ('count', ('VARb', 2)))
        self.assertEqual(batch_request('sort all visible boards'), ('all', None))
        for text in ('do not pick two VARb', 'what if I pick two VARb', 'pick two VARd', 'sort everything now'):
            self.assertIsNone(batch_request(text))

    def test_count_and_all_plans(self):
        for text in ('pick 2 VARb', 'sort all visible boards'):
            self.assertEqual(make_plan(batch_request(text), self.scene).items, ['VARb', 'VARb'])

    def test_invalid_or_unavailable_counts(self):
        for count in (0, 3, 11, 999):
            with self.assertRaises(ValueError):
                make_plan(('count', ('VARb', count)), self.scene)

    def test_bad_detections_never_count(self):
        for field, value in (('inside_calibration', False), ('size_ok', False), ('x', float('nan'))):
            scene = copy.deepcopy(self.scene)
            scene['boards'][0][field] = value
            with self.assertRaises(ValueError):
                visible_counts(scene)
        self.scene['boards'][1]['x'] = .101
        with self.assertRaises(ValueError):
            visible_counts(self.scene)

    def harness(self, phase='completed', remove=True, cancel=False, visual=True):
        source = Path(__file__).resolve().parents[1] / 'bridge.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
        self.now = 100.0
        def sleep(seconds):
            self.now += seconds
            node.scene['stamp'] = self.now
            node.state['stamp'] = self.now
            node.images['overhead'] = (self.now, 'image')
            if cancel:
                node.epoch += 1
        scope = dict(Node=object, copy=copy, json=json, time=SimpleNamespace(
            time=lambda: self.now, monotonic=lambda: self.now, sleep=sleep),
            intent=intent, block_reason=block_reason, visual_agreement=visual_agreement,
            visible_counts=visible_counts, make_plan=make_plan, ordinal=ordinal, uuid=uuid,
            String=lambda **kw: SimpleNamespace(**kw))
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), scope)
        node = scope['Conversation'].__new__(scope['Conversation'])
        node.scene = copy.deepcopy(self.scene)
        node.state = dict(stamp=self.now, hardware='SIMULATION', busy=False, phase='idle', task_id='')
        node.images = {'overhead': (self.now, 'image')}
        node.history = []; node.lock = threading.RLock(); node.epoch = 0
        node.allow_replay = False; node.enable_real_batches = False
        node.busy = False; node.pending = None; node.batch_task_id = None; node.batch_worker_active = True
        node.say = Mock(); node.emit = Mock()
        node.http = Mock(return_value={'target_visible': visual, 'conflict': False})
        def publish(message):
            task = json.loads(message.data)
            node.state.update(task_id=task['id'], phase=phase)
            if remove and phase == 'completed':
                node.scene['boards'].pop(0)
        node.tasks = SimpleNamespace(publish=Mock(side_effect=publish))
        batch = make_plan(('count', ('VARb', 2)), node.scene)
        node.batch = batch
        return node, batch

    def test_two_picks_use_two_visual_checks_and_fresh_removal(self):
        node, batch = self.harness()
        node.run_batch(batch, 0)
        self.assertEqual((batch.phase, batch.completed), ('completed', 2))
        self.assertEqual(node.tasks.publish.call_count, 2)
        self.assertEqual(node.http.call_count, 2)
        self.assertFalse(node.batch_worker_active)
        spoken = [call.args[0] for call in node.say.call_args_list]
        self.assertEqual(spoken, ['Picking the first board, variant B.', 'First board placed.',
                                 'Picking the second board, variant B.', 'Second board placed. All 2 boards are done.'])

    def test_failure_stops_after_one_dispatch(self):
        for phase in ('failed', 'rejected', 'stopped'):
            node, batch = self.harness(phase=phase)
            node.run_batch(batch, 0)
            self.assertEqual(batch.phase, 'failed')
            self.assertEqual(node.tasks.publish.call_count, 1)

    def test_unchanged_scene_cannot_repeat_pick(self):
        node, batch = self.harness(remove=False)
        node.run_batch(batch, 0)
        self.assertEqual(batch.phase, 'failed')
        self.assertEqual(node.tasks.publish.call_count, 1)
        self.assertIn('removal', batch.reason)

    def test_cancel_during_verification_discards_remainder(self):
        node, batch = self.harness(cancel=True)
        node.run_batch(batch, 0)
        self.assertEqual(node.tasks.publish.call_count, 1)
        self.assertNotEqual(batch.phase, 'completed')

    def test_visual_refusal_dispatches_nothing(self):
        node, batch = self.harness(visual=False)
        node.run_batch(batch, 0)
        node.tasks.publish.assert_not_called()
        self.assertEqual(batch.phase, 'failed')

    def test_real_batches_disabled_by_default(self):
        node, _ = self.harness()
        node.batch_worker_active = False
        node.state['hardware'] = 'REAL FR5'
        node.start_batch(('count', ('VARb', 2)))
        node.tasks.publish.assert_not_called()
        self.assertIn('disabled', node.say.call_args.args[0])

    def test_missing_destination_rejects_plan(self):
        self.scene['destinations'] = []
        with self.assertRaises(ValueError):
            make_plan(('all', None), self.scene)

    def test_unrelated_completion_does_not_advance(self):
        node, batch = self.harness()
        node.tasks.publish.side_effect = lambda message: node.state.update(
            task_id='another-task', phase='completed')
        node.run_batch(batch, 0)
        self.assertEqual(node.tasks.publish.call_count, 1)
        self.assertEqual(batch.completed, 0)
        self.assertIn('acknowledgement', batch.reason)

    def test_busy_batch_rejects_another_request(self):
        node, _ = self.harness()
        node.start_batch(('count', ('VARb', 2)))
        node.tasks.publish.assert_not_called()
        self.assertIn('already active', node.say.call_args.args[0])

    def test_scene_movement_during_visual_check_blocks_dispatch(self):
        node, batch = self.harness()
        def moved(*args, **kwargs):
            node.scene['boards'][0]['x'] += .02
            return {'target_visible': True, 'conflict': False}
        node.http.side_effect = moved
        node.run_batch(batch, 0)
        node.tasks.publish.assert_not_called()

    def test_stale_scene_rejected_before_start(self):
        node, _ = self.harness()
        node.batch_worker_active = False
        node.scene['stamp'] = 90
        node.start_batch(('count', ('VARb', 2)))
        node.tasks.publish.assert_not_called()
        self.assertIn('stale', node.say.call_args.args[0])
