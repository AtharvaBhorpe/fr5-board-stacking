"""Exercise the pick script's service failure paths without importing its robot entrypoint."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class SceneServiceTimeoutTest(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[2] / "robot/pick_one.py"
        function = next(n for n in ast.parse(source.read_text()).body
                        if isinstance(n, ast.FunctionDef) and n.name == "call")
        scope = {"node": object(), "rclpy": SimpleNamespace(spin_until_future_complete=Mock())}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
        self.call = scope["call"]

    def test_missing_service_sends_no_request(self):
        client = Mock()
        client.wait_for_service.return_value = False
        with self.assertRaisesRegex(SystemExit, "unavailable"):
            self.call(client, object())
        client.call_async.assert_not_called()

    def test_lost_response_cancels_future_and_stops(self):
        client = Mock()
        future = client.call_async.return_value
        future.done.return_value = False
        with self.assertRaisesRegex(SystemExit, "timed out"):
            self.call(client, object())
        future.cancel.assert_called_once()
        future.result.assert_not_called()

    def test_completed_response_is_returned(self):
        client = Mock()
        self.assertIs(self.call(client, object()), client.call_async.return_value.result.return_value)
