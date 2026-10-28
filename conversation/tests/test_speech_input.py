import ast
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from speech_input import normalize_voice_command, transcribe_audio
from policy import batch_request


class SpeechInputTest(unittest.TestCase):
    def test_observed_error_preserves_two_and_variant_b(self):
        text = normalize_voice_command('Pick two variant B volts.')
        self.assertEqual(text, 'Pick two variant B boards.')
        self.assertEqual(batch_request(text), 'transfer VARb drop_off_b 2')

    def test_no_invented_counts_or_actions(self):
        for text in ('Pick variant B.', 'Do not pick two variant B volts.',
                     'What if I pick two variant B volts?', 'Set two volts.',
                     'Pick two variant D volts.', 'Pick two variant B volts and release vacuum.'):
            self.assertEqual(normalize_voice_command(text), text)

    def test_parakeet_joined_variant_preserves_command(self):
        self.assertEqual(batch_request(normalize_voice_command('Pick three variant aboards.')),
                         'transfer VARa drop_off_a 3')
        for text in ('Do not pick three variant aboards.', 'Pick variant aboards.',
                     'Pick three variant dboards.', 'Could I pick three variant aboards?'):
            self.assertEqual(normalize_voice_command(text), text)

    def test_shared_asr_has_counted_vocabulary_and_beam_search(self):
        asr = Mock()
        asr.transcribe.return_value = (iter([SimpleNamespace(text='Pick two variant B boards.')]), None)
        self.assertEqual(transcribe_audio(asr, object()), 'Pick two variant B boards.')
        options = asr.transcribe.call_args.kwargs
        self.assertEqual(options['beam_size'], 5)
        self.assertIn('two variant B boards', options['initial_prompt'])

    def test_spoken_messages_are_fifo_and_stale_messages_are_discarded(self):
        source = Path(__file__).resolve().parents[1] / 'bridge.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
        loop = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'speech_loop')
        scope = {'time': SimpleNamespace(monotonic=lambda: 0, sleep=Mock())}
        exec(compile(ast.Module(body=[loop], type_ignores=[]), str(source), 'exec'), scope)
        inbox = Mock()
        inbox.get.side_effect = [('old', 0), ('First board placed.', 1),
                                 ('Picking the second board.', 1), StopIteration]
        node = SimpleNamespace(speech_queue=inbox, epoch=1, lock=threading.RLock(),
                               http=Mock(return_value={'active': False}), emit=Mock())
        with self.assertRaises(StopIteration):
            scope['speech_loop'](node)
        spoken = [c.args[1]['text'] for c in node.http.call_args_list if c.args[0] == '/speak']
        self.assertEqual(spoken, ['First board placed.', 'Picking the second board.'])

    def test_interruption_silences_previous_utterance_before_new_one(self):
        source = Path(__file__).resolve().parents[1] / 'bridge.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
        loop = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'speech_loop')
        scope = {'time': SimpleNamespace(monotonic=lambda: 0, sleep=Mock())}
        exec(compile(ast.Module(body=[loop], type_ignores=[]), str(source), 'exec'), scope)
        inbox = Mock()
        inbox.get.side_effect = [('Picking the first board.', 1), ('Cancellation requested.', 2), StopIteration]
        node = SimpleNamespace(speech_queue=inbox, epoch=1, lock=threading.RLock(), emit=Mock())
        def http(path, body, **kwargs):
            if path == '/speak' and body['text'].startswith('Picking'):
                node.epoch = 2
            return {'active': False}
        node.http = Mock(side_effect=http)
        with self.assertRaises(StopIteration):
            scope['speech_loop'](node)
        self.assertEqual([c.args[0] for c in node.http.call_args_list],
                         ['/speak', '/silence', '/speak', '/speech/status'])
