import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

spec = importlib.util.spec_from_file_location('sorting_observer', Path(__file__).resolve().parents[1]/'scripts/sorting_observer.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class WorkerProtocolTests(unittest.TestCase):
    def test_json_reply(self):
        with subprocess.Popen([sys.executable, '-c', 'print(\'{"ready": true}\', flush=True)'],
                              stdout=subprocess.PIPE, text=True) as p:
            self.assertEqual(m.read_reply(p, 3), {'ready': True})

    def test_exited_worker(self):
        with subprocess.Popen([sys.executable, '-c', 'pass'], stdout=subprocess.PIPE, text=True) as p:
            with self.assertRaisesRegex(RuntimeError, 'exited'):
                m.read_reply(p, 3)

    def test_timeout_kills_worker(self):
        with subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(5)'],
                              stdout=subprocess.PIPE, text=True) as p:
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                m.read_reply(p, .05)
            p.wait(timeout=3)
            self.assertNotEqual(p.returncode, 0)


if __name__ == '__main__': unittest.main()
