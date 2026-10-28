"""Conversation unit tests: no ROS, no GPU, no model.   pixi run python -m unittest discover -s conversation/tests"""
import ast
import copy
import io
import json
import re
import sys
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from policy import batch_request  # noqa: E402
from speech_input import normalize_voice_command  # noqa: E402


class VoiceRepair(unittest.TestCase):
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


def model_server_functions(*names):
    """Load functions from model_server.py without importing ptts / torch."""
    source = HERE / "model_server.py"
    nodes = [n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name in names]
    scope = {"json": json, "re": re, "urllib": urllib, "LLM": "http://localhost/model"}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), scope)
    return scope


class SpokenReply(unittest.TestCase):
    def test_speech_prompt_keeps_original_log_and_normal_answers(self):
        ask = model_server_functions("ask")["ask"]
        requests = []

        def respond(request, **kwargs):
            requests.append(json.loads(request.data))
            return io.BytesIO(json.dumps({"choices": [{"message": {"content": "The robot is paused."}}]}).encode())

        body = {"text": "Controller paused: j2 queued, 0 delivered.", "speech_only": True}
        original = copy.deepcopy(body)
        with patch("urllib.request.urlopen", side_effect=respond):
            self.assertEqual(ask(body)["answer"], "The robot is paused.")
            ask({"text": "What can you see?", "images": ["abc"]})
            ask({**body, "language": "de"})
        self.assertEqual(body, original)
        prompt = requests[0]["messages"][0]["content"][0]["text"]
        self.assertIn("Queued means waiting, not moving or completed", prompt)
        self.assertIn("under 20 words", prompt)
        self.assertEqual(json.loads(requests[0]["messages"][-1]["content"][-1]["text"])["question"], original["text"])
        self.assertNotIn("Rewrite the supplied robot log", requests[1]["messages"][0]["content"][0]["text"])
        self.assertEqual(requests[1]["messages"][-1]["content"][0]["image_url"]["url"], "data:image/jpeg;base64,abc")
        self.assertIn("entirely in natural German", requests[2]["messages"][0]["content"][0]["text"])

    def test_variant_names_are_spoken(self):
        spoken = model_server_functions("spoken_text")["spoken_text"]
        self.assertEqual(spoken("Job j1: transfer VARb, then VARa/c"), "Job j1: transfer variant B, then variant A, variant C")


if __name__ == "__main__":
    unittest.main()
