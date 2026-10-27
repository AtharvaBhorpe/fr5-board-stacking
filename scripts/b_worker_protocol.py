"""Read one JSON response from the persistent GPU worker."""
import json
import math
import os
import select
import time


def read_reply(process, timeout):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Worker timeout must be finite and positive')
    deadline = time.monotonic() + timeout
    data = bytearray()
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
                raise RuntimeError('GPU worker timed out; restart perception')
            chunk = os.read(process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError('GPU worker exited; inspect its error output')
            data.extend(chunk)
            if len(data) > 4 * 1024 * 1024:
                raise RuntimeError('GPU worker response exceeds 4 MiB')
            if b'\n' in data:
                line, trailing = data.split(b'\n', 1)
                if trailing.strip():
                    raise RuntimeError('Unexpected extra GPU worker response')
                reply = json.loads(line)
                if not isinstance(reply, dict):
                    raise ValueError('GPU worker response must be a JSON object')
                return reply
    except Exception:
        # A partial/bad response loses framing; never reuse this worker for another image.
        if process.poll() is None:
            process.kill()
        raise
