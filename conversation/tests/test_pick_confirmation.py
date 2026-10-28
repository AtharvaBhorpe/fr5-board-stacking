"""Verify descent sequencing without loading ROS or commanding hardware."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class PickConfirmationTest(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[2] / "robot/pick_one.py"
        function = next(n for n in ast.parse(source.read_text()).body
                        if isinstance(n, ast.FunctionDef) and n.name == "transfer")
        result = SimpleNamespace(success=True, message="ok", time_to_vacuum=0.1)
        self.events = []
        self.scope = {
            "math": math, "print": Mock(), "tool_yaw": lambda b: 0.0,
            "args": SimpleNamespace(speed=0.1, hover_only=False, place_press=1.0, blow=2.0,
                                    fixed_pick_height=False, press=4.0),
            "HOVER": 0.1, "PRESS": 0.004, "wd": None,
            "MoveTo": SimpleNamespace(Goal=lambda **kw: kw), "move_to": "move",
            "Grip": SimpleNamespace(Goal=lambda: None), "grip": "grip",
            "act": Mock(return_value=result),
            "check": lambda label, result: self.events.append(label),
            "lin": Mock(return_value=result),
            "SIZES": {"VARb": (0.15, 0.21)}, "BOARD_H": 0.005,
            "call": Mock(), "attach": "attach",
            "AttachBoard": SimpleNamespace(Request=lambda **kw: kw),
            "DetachBoard": SimpleNamespace(Request=lambda **kw: kw), "detach": "detach",
            "Release": SimpleNamespace(Goal=lambda **kw: kw), "release": "release",
            "time": SimpleNamespace(time=lambda: 123.0),
            "input": Mock(side_effect=EOFError("placement confirmation")),
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), self.scope)
        self.board = dict(variant="VARb", x=0.0, y=0.0, z=0.0, yaw=0.0,
                          size_mm=[150, 210], inside_calibration=True)

    def test_pick_and_place_finish_without_confirmation(self):
        self.assertEqual(self.scope["transfer"](self.board, to=(0.2, 0.3, 0.0, 0.0)), 123.0)
        self.assertEqual(self.events, ["hover", "down", "lift", "carry", "down again", "lift empty"])
        self.scope["input"].assert_not_called()
        self.assertEqual([c.args[0] for c in self.scope["act"].call_args_list],
                         ["move", "grip", "move", "release"])

    def test_failed_carry_or_placement_never_releases(self):
        for failed_step in ("carry", "down again"):
            with self.subTest(step=failed_step):
                self.scope["act"].reset_mock()
                def check(label, result):
                    if label == failed_step:
                        raise SystemExit("motion failed")
                self.scope["check"] = check
                with self.assertRaises(SystemExit):
                    self.scope["transfer"](self.board, to=(0.2, 0.3, 0.0, 0.0))
                self.assertNotIn("release", [c.args[0] for c in self.scope["act"].call_args_list])

    def test_hover_only_never_descends(self):
        self.scope["args"].hover_only = True
        with self.assertRaises(SystemExit):
            self.scope["transfer"](self.board)
        self.scope["lin"].assert_not_called()
        self.scope["input"].assert_not_called()

    def test_fixed_pick_uses_configured_height_without_wrist_measurement(self):
        wrist = Mock()
        self.scope["wd"] = wrist
        self.scope["args"].fixed_pick_height = True
        self.scope["args"].press = 3.0
        self.scope["PRESS"] = 0.003
        self.scope["transfer"](self.board, to=(0.2, 0.3, 0.0, 0.0))
        wrist.measure.assert_not_called()
        self.assertAlmostEqual(self.scope["lin"].call_args_list[0].args[0], -0.103)
        self.assertIs(self.scope["wd"], wrist)

    def test_failed_hover_never_descends(self):
        self.scope["check"] = Mock(side_effect=SystemExit("hover failed"))
        with self.assertRaises(SystemExit):
            self.scope["transfer"](self.board)
        self.scope["lin"].assert_not_called()
        self.scope["input"].assert_not_called()
