import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import sys
import threading
import uuid

from demo_chat import parse_command


ROOT = Path(__file__).resolve().parents[1]
LOCK = threading.Lock()
STATE = {
    "job_id": None,
    "status": "idle",
    "variant": None,
    "step": None,
    "events": [],
    "output": [],
}

PAGE = r"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FR5 Processing Demo</title>
<style>
body{font:16px system-ui;background:#101820;color:#e9eef2;
max-width:1000px;margin:40px auto;padding:0 20px}
h1{margin-bottom:8px}.badge{color:#ffd479}
section{background:#1b2733;padding:22px;border-radius:12px;margin:20px 0}
input,button{font:inherit;padding:12px;border-radius:6px;border:1px solid #607080}
input{width:min(60%,450px)}button{cursor:pointer;background:#a8ddd0}
button:disabled{opacity:.5;cursor:wait}
pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:350px;overflow:auto}
#error{color:#ffaaa0}dt{color:#aebdcc}dd{margin:4px 0 14px}
</style>
<h1>FR5 board processing</h1>
<div class="badge">FAKE HARDWARE · SYNTHETIC POSES AND OBSERVATIONS</div>
<p>Commands run through MoveIt and the fake gripper. Physical board handling
and camera observations are not connected.</p>

<section>
<form id="form">
<label for="command">Processing command</label><br><br>
<input id="command" value="process VARa" autocomplete="off">
<button id="send" type="submit">Run fake job</button>
</form>
<p>Accepted: process VARa · process VARb · process VARc</p>
<p id="error" role="alert"></p>
</section>

<section>
<dl>
<dt>Job status</dt><dd id="status">Connecting…</dd>
<dt>Variant</dt><dd id="variant">—</dd>
<dt>Current / last step</dt><dd id="step">—</dd>
<dt>Job ID</dt><dd id="job">—</dd>
</dl>
</section>

<section><h2>Job events</h2><pre id="events"></pre></section>
<details><summary>ROS and runner output</summary><pre id="output"></pre></details>

<script>
const byId = id => document.getElementById(id);
let submitting = false;
let active = false;

async function refresh() {
  try {
    const response = await fetch('/api/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('Status request failed');
    const state = await response.json();
    active = state.status === 'running';
    byId('status').textContent = state.status;
    byId('variant').textContent = state.variant || '—';
    byId('step').textContent = state.step || '—';
    byId('job').textContent = state.job_id || '—';
    byId('events').textContent = state.events.map(e =>
      `${e.elapsed_s ?? ''}  ${e.event}  ${e.step || ''}` +
      (e.reason ? ` — ${e.reason}` : '')
    ).join('\n');
    byId('output').textContent = state.output.join('\n');
    byId('send').disabled = active || submitting;
  } catch (error) {
    byId('status').textContent = 'Server unavailable; job state unknown';
    byId('send').disabled = true;
  }
}
byId('form').addEventListener('submit', async event => {
  event.preventDefault();
  if (submitting || active) return;
  submitting = true;
  byId('send').disabled = true;
  byId('error').textContent = '';
  try {
    const response = await fetch('/api/job', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: byId('command').value})
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || 'Job rejected');
  } catch (error) {
    byId('error').textContent = error.message;
  } finally {
    submitting = false;
    await refresh();
  }
});
async function poll() {
  await refresh();
  setTimeout(poll, 500);
}
poll();
</script>
</html>
"""


def collect_output(process):
    completed = False
    try:
        for raw in process.stdout:
            line = raw.rstrip()
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                event = None

            with LOCK:
                STATE["output"].append(line)
                STATE["output"] = STATE["output"][-300:]
                if isinstance(event, dict) and "event" in event:
                    STATE["events"].append(event)
                    if event.get("step"):
                        STATE["step"] = event["step"]
                    if event["event"] == "SIMULATED_JOB_COMPLETED":
                        completed = True

        code = process.wait()
        with LOCK:
            STATE["status"] = (
                "completed (fake)" if code == 0 and completed else "faulted"
            )
    except Exception as error:
        # Keep the slot occupied until the child actually exits.
        with LOCK:
            STATE["output"].append(f"Output collection failed: {error}")
        process.wait()
        with LOCK:
            STATE["status"] = "faulted"
    finally:
        process.stdout.close()


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value):
        body = json.dumps(value).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def trusted_host(self):
        return self.headers.get("Host") == self.server.local_authority

    def do_GET(self):
        if not self.trusted_host():
            return self.reply(403, {"error": "Use the printed local URL"})
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            with LOCK:
                snapshot = json.loads(json.dumps(STATE))
            self.reply(200, snapshot)
        else:
            self.reply(404, {"error": "Not found"})

    def do_POST(self):
        expected_origin = f"http://{self.server.local_authority}"
        if (
            not self.trusted_host()
            or self.headers.get("Origin") != expected_origin
        ):
            return self.reply(403, {"error": "Request must come from this page"})
        if self.path != "/api/job":
            return self.reply(404, {"error": "Not found"})
        if self.headers.get_content_type() != "application/json":
            return self.reply(415, {"error": "Expected JSON"})

        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 1024:
                raise ValueError("Invalid request size")
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("Expected a command object")
            command = payload.get("command")
            if not isinstance(command, str):
                raise ValueError("Command must be text")
            variant = parse_command(command)
        except (ValueError, UnicodeError) as error:
            return self.reply(400, {"error": str(error)})

        with LOCK:
            if STATE["status"] == "running":
                error = (409, {"error": "A job is already running"})
            else:
                job_id = uuid.uuid4().hex
                try:
                    process = subprocess.Popen(
                        [
                            sys.executable, "-u",
                            str(ROOT / "scripts/run_fake_ros_job.py"),
                            "--variant", variant,
                        ],
                        cwd=ROOT,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        errors="replace",
                    )
                except OSError as exc:
                    error = (500, {"error": f"Could not start runner: {exc}"})
                else:
                    STATE.update({
                        "job_id": job_id,
                        "status": "running",
                        "variant": variant,
                        "step": None,
                        "events": [],
                        "output": [],
                    })
                    threading.Thread(
                        target=collect_output, args=(process,), daemon=False
                    ).start()
                    error = None

        if error:
            return self.reply(*error)
        self.reply(202, {"job_id": job_id, "variant": variant})

    def log_message(self, format, *args):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.local_authority = f"127.0.0.1:{server.server_port}"
    print(f"Open http://{server.local_authority}")
    print("Fake ROS only. Keep this terminal and pixi run demo running.")
    print("Closing the browser does not cancel a job.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nWeb server closing. Any active runner is allowed to finish.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
