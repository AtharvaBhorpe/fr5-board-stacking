import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace


spec = importlib.util.spec_from_file_location(
    'checks', Path(__file__).parents[1] / 'scripts' / 'check_perception_hardware.py')
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)


class HardwareChecksTest(unittest.TestCase):
    def test_missing_cuda_is_failure_instead_of_cpu_fallback(self):
        fake_torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False))
        with patch.dict('sys.modules', {'torch': fake_torch}):
            with self.assertRaisesRegex(RuntimeError, 'cannot access CUDA'):
                checks.cuda()

    def test_empty_webcam_selection_never_opens_laptop_camera(self):
        fake_cv = SimpleNamespace(VideoCapture=unittest.mock.Mock())
        with patch.dict('sys.modules', {'cv2': fake_cv}):
            with self.assertRaisesRegex(ValueError, 'Connect the Logitech'):
                checks.logitech({'logitech': {'device': ''}}, None)
        fake_cv.VideoCapture.assert_not_called()

    def test_failed_webcam_read_releases_camera(self):
        cap = unittest.mock.Mock()
        cap.isOpened.return_value = True
        cap.read.return_value = (False, None)
        fake_cv = SimpleNamespace(VideoCapture=unittest.mock.Mock(return_value=cap), CAP_V4L2=200)
        with tempfile.NamedTemporaryFile() as device:
            with patch.dict('sys.modules', {'cv2': fake_cv}):
                with self.assertRaisesRegex(RuntimeError, 'did not deliver'):
                    checks.logitech({'logitech': {'device': device.name}, 'probe': {'frames': 2}}, None)
        cap.release.assert_called_once()

    def test_inventory_reports_missing_driver_without_opening_cameras(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            usb = root / 'usb' / 'camera'
            usb.mkdir(parents=True)
            (usb / 'idVendor').write_text('046d')
            (usb / 'product').write_text('Logitech')
            (usb / 'serial').write_text('must-not-be-reported')
            config = {'host': {'usb_root': str(root / 'usb'), 'video_root': str(root / 'video'),
                               'driver_version_file': str(root / 'missing-driver')}}
            with patch.object(checks.subprocess, 'run', side_effect=FileNotFoundError('missing')):
                report = checks.inventory(config)
            self.assertFalse(report['gpu']['ok'])
            self.assertFalse(report['nvidia_driver_loaded'])
            self.assertEqual(report['usb_devices'], [{'idVendor': '046d', 'product': 'Logitech'}])
            self.assertEqual(report['video_devices'], [])

    def test_camera_timeout_returns_failure(self):
        with patch.object(checks.subprocess, 'run', side_effect=subprocess.TimeoutExpired('probe', 1)):
            self.assertEqual(checks.main(['d405']), 2)

    def test_invalid_frame_count_does_not_start_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'invalid.toml'
            config.write_text('[probe]\nframes = 0\ntimeout_seconds = 2\n')
            with patch.object(checks.subprocess, 'run') as run:
                self.assertEqual(checks.main(['d405', '--config', str(config)]), 2)
            run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
