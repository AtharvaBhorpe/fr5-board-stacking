"""Shared test setup: installed paths (via ament index, no hard-coded workspace) + log directory.

Importing this module changes the working directory to the log directory, so every script writes
its logs there: $HK_TEST_LOG_DIR, default /tmp/hk_fr5_tests.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
MOCK = os.path.join(HERE, "mock_fairino.py")  # mock Fairino XML-RPC server (tool DO / DI)
SHELL = os.path.join(get_package_prefix("hk_fr5_motion"), "lib", "hk_fr5_motion", "motion_shell")
GRIPPER_CFG = os.path.join(get_package_share_directory("hk_fr5_gripper"), "config", "gripper.yaml")
SCENE_CFG = os.path.join(get_package_share_directory("hk_fr5_motion"), "config", "scene.yaml")
LOG_DIR = os.environ.get("HK_TEST_LOG_DIR", "/tmp/hk_fr5_tests")
os.makedirs(LOG_DIR, exist_ok=True)
os.chdir(LOG_DIR)
