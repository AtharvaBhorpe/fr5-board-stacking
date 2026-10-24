"""Read-only inventory and bounded camera-stream checks. Never connects to a robot."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import tomllib


def inventory(config):
    usb = []
    for entry in sorted(Path(config['host']['usb_root']).glob('*')):
        fields = {}
        for key in ('idVendor', 'idProduct', 'product'):
            path = entry / key
            if path.is_file():
                fields[key] = path.read_text().strip()
        if 'idVendor' in fields:
            usb.append(fields)
    try:
        gpu = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader'],
            capture_output=True, text=True, timeout=10,
        )
        gpu_info = {'ok': gpu.returncode == 0, 'detail': (gpu.stdout + gpu.stderr).strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        gpu_info = {'ok': False, 'detail': str(exc)}
    return {
        'gpu': gpu_info,
        'nvidia_driver_loaded': Path(config['host']['driver_version_file']).is_file(),
        'video_devices': [entry.name for entry in sorted(Path(config['host']['video_root']).glob('*'))],
        'usb_devices': usb,
        'note': 'Inventory does not open any camera or verify a stream.',
    }


def logitech(config, device, capture_dir=None):
    import cv2

    device = device or config['logitech']['device']
    if not device:
        raise ValueError('Connect the Logitech and supply --device or set logitech.device in config.')
    if not Path(device).exists():
        raise ValueError(f'Camera device does not exist: {device}')
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open selected webcam; check permissions and other camera applications.')
        settings = config['logitech']
        if settings.get('fourcc'):
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings['fourcc']))
        if 'autofocus' in settings:
            if not cap.set(cv2.CAP_PROP_AUTOFOCUS, int(settings['autofocus'])):
                raise RuntimeError('Camera refused the configured autofocus mode.')
        if 'focus_position' in settings:
            if not cap.set(cv2.CAP_PROP_FOCUS, settings['focus_position']):
                raise RuntimeError('Camera refused the configured fixed focus.')
        for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT),
                          ('fps', cv2.CAP_PROP_FPS)):
            if settings.get(key):
                cap.set(prop, settings[key])
        received = []
        for _ in range(config['probe']['frames']):
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                raise RuntimeError('Webcam opened but did not deliver the requested frames.')
            received.append(time.monotonic())
        report = {'ok': True, 'frames': config['probe']['frames'], 'width': frame.shape[1],
                'height': frame.shape[0], 'reported_fps': cap.get(cv2.CAP_PROP_FPS),
                'note': 'Frame delivery passed; focus, exposure and calibration still require visual checks.'}
        if len(received) > 1 and received[-1] > received[0]:
            report['observed_read_fps'] = (len(received) - 1) / (received[-1] - received[0])
        if capture_dir:
            folder = Path(capture_dir)
            folder.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(folder / 'logitech_rgb.png'), frame):
                raise RuntimeError('Failed to save Logitech sample.')
            (folder / 'logitech_metadata.json').write_text(json.dumps(report, indent=2))
        return report
    finally:
        cap.release()


def d405(config, capture_dir=None):
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    devices = [dev for dev in rs.context().query_devices()
               if 'D405' in dev.get_info(rs.camera_info.name)]
    requested = config['d405']['serial']
    if requested:
        devices = [dev for dev in devices if dev.get_info(rs.camera_info.serial_number) == requested]
    if len(devices) != 1:
        raise ValueError(f'Expected exactly one selected D405, found {len(devices)}. Connect it or set d405.serial.')
    stream_config = rs.config()
    stream_config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
    # Use device-supported defaults rather than guessing profiles from another model.
    stream_config.enable_stream(rs.stream.depth)
    stream_config.enable_stream(rs.stream.color)
    pipeline = rs.pipeline()
    profile = pipeline.start(stream_config)
    try:
        for _ in range(config['probe']['frames']):
            frames = pipeline.wait_for_frames(timeout_ms=3000)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                raise RuntimeError('D405 did not deliver both color and depth.')
        image = np.asanyarray(depth.get_data())
        valid_fraction = float(np.count_nonzero(image) / image.size)
        report = {'ok': True, 'frames': config['probe']['frames'],
                'depth_size': [depth.get_width(), depth.get_height()],
                'color_size': [color.get_width(), color.get_height()],
                'depth_scale_m': profile.get_device().first_depth_sensor().get_depth_scale(),
                'nonzero_depth_fraction_last_frame': valid_fraction,
                'note': 'Stream delivery passed; nonzero depth is not an accuracy or grasp-readiness check.'}
        if capture_dir:
            folder = Path(capture_dir)
            folder.mkdir(parents=True, exist_ok=True)
            aligned = rs.align(rs.stream.color).process(frames)
            aligned_depth = aligned.get_depth_frame()
            rgb = np.asanyarray(color.get_data())
            fmt = color.get_profile().format()
            if fmt == rs.format.rgb8:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif fmt != rs.format.bgr8:
                raise RuntimeError(f'Unsupported color format for saved sample: {fmt}')
            intrinsics = aligned_depth.profile.as_video_stream_profile().get_intrinsics()
            report['aligned_depth_intrinsics'] = {
                key: getattr(intrinsics, key) for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')}
            report['aligned_depth_intrinsics']['model'] = str(intrinsics.model)
            report['aligned_depth_intrinsics']['coeffs'] = list(intrinsics.coeffs)
            report['camera_timestamp_ms'] = depth.get_timestamp()
            report['timestamp_domain'] = str(depth.get_frame_timestamp_domain())
            if not cv2.imwrite(str(folder / 'd405_rgb.png'), rgb):
                raise RuntimeError('Failed to save D405 color sample.')
            np.save(folder / 'd405_raw_depth.npy', image)
            np.save(folder / 'd405_aligned_depth.npy', np.asanyarray(aligned_depth.get_data()))
            (folder / 'd405_metadata.json').write_text(json.dumps(report, indent=2))
        return report
    finally:
        pipeline.stop()


def cuda():
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError('PyTorch cannot access CUDA. Check the driver and use the perception-gpu environment.')
    with torch.inference_mode():
        values = torch.arange(4096, dtype=torch.float32, device='cuda').reshape(64, 64)
        result = values @ torch.eye(64, dtype=torch.float32, device='cuda')
        torch.cuda.synchronize()
        expected = torch.arange(4096, dtype=torch.float32).reshape(64, 64)
        if not torch.equal(result.cpu(), expected):
            raise RuntimeError('CUDA computation returned an incorrect result.')
    free, total = torch.cuda.mem_get_info()
    return {'ok': True, 'torch_version': torch.__version__, 'torch_cuda_runtime': torch.version.cuda,
            'device': torch.cuda.get_device_name(), 'total_memory_bytes': total,
            'free_memory_bytes': free, 'matrix_multiplication_verified': True,
            'note': 'Numerical CUDA test passed; model memory and latency still require separate measurement.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('inventory', 'logitech', 'd405', 'cuda'))
    parser.add_argument('--config', default='config/perception.toml')
    parser.add_argument('--device', help='Explicit Logitech device; never automatically select the laptop camera.')
    parser.add_argument('--capture-dir', help='Save local RGB/depth samples for visual inspection.')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        with open(args.config, 'rb') as source:
            config = tomllib.load(source)
        if config['probe']['frames'] < 1 or config['probe']['timeout_seconds'] <= 0:
            raise ValueError('Probe frames and timeout_seconds must be positive.')
        if args.kind == 'inventory':
            report = inventory(config)
        elif not args.worker:
            command = [sys.executable, str(Path(__file__).resolve()), args.kind,
                       '--worker', '--config', args.config]
            if args.device:
                command += ['--device', args.device]
            if args.capture_dir:
                command += ['--capture-dir', args.capture_dir]
            # A camera driver can block inside read(); isolate it behind a process timeout.
            result = subprocess.run(command, timeout=config['probe']['timeout_seconds'])
            return result.returncode
        else:
            if args.kind == 'cuda':
                report = cuda()
            else:
                report = (logitech(config, args.device, args.capture_dir) if args.kind == 'logitech'
                          else d405(config, args.capture_dir))
        print(json.dumps(report, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, KeyError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}, indent=2))
        return 2


if __name__ == '__main__':
    sys.exit(main())
