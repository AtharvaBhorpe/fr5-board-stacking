"""Read one JSON response from the persistent GPU worker."""
import json
import select


def read_reply(process, timeout):
    ready, _, _ = select.select([process.stdout], [], [], timeout)
    if not ready:
        process.kill()
        raise RuntimeError('GPU worker timed out; restart live B')
    line = process.stdout.readline()
    if not line:
        raise RuntimeError('GPU worker exited; inspect its error output')
    return json.loads(line)
