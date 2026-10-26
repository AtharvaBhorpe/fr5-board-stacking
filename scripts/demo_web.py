from web_style import STYLE

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
<style>__SHARED_STYLE__
.workspace{padding:0 32px;display:grid;grid-template-columns:minmax(300px, .85fr) minmax(400px,1.4fr);gap:24px;max-width:1500px;margin:auto}.intro{max-width:1500px;margin:auto}.command-row{display:flex;gap:10px;margin-top:12px;flex-wrap:wrap}input{width:100%;min-width:0;flex:1 1 180px}.help{font-size:14px;margin-top:14px}.status-panel{margin-top:24px}dl{margin:0}dt{font-size:14px;color:var(--muted);margin-top:20px}dd{margin:4px 0;overflow-wrap:anywhere}#status{font-size:28px;font-weight:650;letter-spacing:-.8px}#job{font-size:14px}#events{max-height:560px;overflow:auto;min-height:260px}.event{display:grid;grid-template-columns:68px 1fr;gap:16px;padding:14px 0}.event:nth-child(even){background:var(--ground)}.event time{font-size:14px;color:var(--muted);font-variant-numeric:tabular-nums}.event strong{display:block;font-size:15px}.event span{font-size:14px;color:var(--muted);overflow-wrap:anywhere}.event.fault strong{color:var(--error)}.empty-events{padding:55px 12px;color:var(--muted);text-align:center}.output-panel{grid-column:1/-1}#connection{font-size:14px;color:var(--muted)}@media(max-width:850px){.workspace{grid-template-columns:1fr;padding:0 20px}.output-panel{grid-column:auto}#events{max-height:400px}}.replay-panel{grid-column:1/-1}.replay-grid{display:grid;grid-template-columns:1fr 1fr;gap:24px}.replay-meta{display:flex;gap:32px;flex-wrap:wrap;margin-bottom:20px}.replay-meta dt{margin-top:8px}.replay-grid h3{font-size:16px;margin:8px 0}.replay-grid ul{padding-left:22px;margin:8px 0;overflow-wrap:anywhere}.replay-grid li{margin:8px 0}#replay-note{font-size:14px}@media(max-width:700px){.replay-grid{grid-template-columns:1fr;gap:12px}}</style>
<body>
<nav class="topbar" aria-label="Workspace"><div class="brand">FR5 / Workspace</div><span>Job console</span></nav>
<header class="intro"><div><h1>Board processing.</h1><p>Run a variant through the coordinator and follow each operation as it completes.</p></div><div class="notice">Fake hardware only<br>Synthetic poses &amp; observations</div></header>
<main class="workspace"><div><section class="panel"><h2>Start a job</h2>
<form id="form"><label for="command">Processing command</label><div class="command-row"><input id="command" value="process VARa" autocomplete="off" aria-describedby="command-help"><button id="send" type="submit" disabled>Run fake job</button></div></form>
<p class="help" id="command-help">Use process VARa, process VARb, or process VARc.</p><p id="error" role="alert"></p></section>
<section class="panel status-panel"><h2>Simulated job state</h2><dl><dt>Status</dt><dd id="status" role="status">Connecting…</dd><dt>Variant</dt><dd id="variant">—</dd><dt>Current / last step</dt><dd id="step">—</dd><dt>Job ID</dt><dd id="job">—</dd></dl></section></div>
<section class="panel"><div class="section-head"><h2>Execution timeline</h2><span id="connection">Connecting</span></div><div id="events"></div></section>
<section class="panel replay-panel" aria-labelledby="replay-title">
<div class="section-head"><h2 id="replay-title">Historical replay</h2><span id="replay-connection" role="status">Disconnected</span></div>
<p>Recorded-image observations, separate from simulated job state. Replay never enables execution.</p>
<dl class="replay-meta"><div><dt>Stream state</dt><dd id="replay-stream">Unknown</dd></div><div><dt>Age since bridge receipt</dt><dd id="replay-age">Unknown</dd></div></dl>
<p id="replay-note">Historical image capture age is unknown. Waiting for scene data.</p>
<div class="replay-grid"><div><h3>Destination tags</h3><ul id="replay-tags"><li>Unavailable</li></ul></div><div><h3>Source candidates</h3><ul id="replay-candidates"><li>Unavailable</li></ul></div></div>
</section>
<details class="panel output-panel"><summary>ROS &amp; runner output</summary><pre id="output"></pre></details></main>
<footer>Physical board handling is not connected. Perception shown here is historical replay only. Closing this page does not cancel a running job.</footer>
<noscript>JavaScript is required to submit a command and view job state.</noscript>
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
    byId('connection').textContent = 'Connected';
    const signature=JSON.stringify(state.events);
    if(byId('events').dataset.signature!==signature){
      byId('events').dataset.signature=signature;
      byId('events').replaceChildren();
      if(!state.events.length){const empty=document.createElement('p');empty.className='empty-events';empty.textContent='No events yet. Start a fake job to follow its progress.';byId('events').append(empty);}
      for(const e of state.events){
        const row=document.createElement('div');row.className='event'+(e.event.includes('FAULT')?' fault':'');
        const time=document.createElement('time');time.textContent=Number.isFinite(e.elapsed_s)?`${e.elapsed_s.toFixed(2)} s`:'—';
        const content=document.createElement('div');const title=document.createElement('strong');title.textContent=e.event.toLowerCase().replaceAll('_',' ');
        const detail=document.createElement('span');detail.textContent=[e.step,e.reason].filter(Boolean).join(' — ');
        content.append(title,detail);row.append(time,content);byId('events').append(row);
      }
    }
    byId('output').textContent = state.output.join('\n');
    byId('send').disabled = active || submitting;
  } catch (error) {
    byId('connection').textContent = 'Disconnected';
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
// Replay is display-only and has no access to the job submission state.
function replayList(id, values, emptyText) {
  const list = byId(id);
  list.replaceChildren();
  for (const value of values.length ? values : [emptyText]) {
    const item = document.createElement('li');
    item.textContent = value;
    list.append(item);
  }
}
function disconnectReplay(reason, stream = 'Unknown', age = null) {
  byId('replay-connection').textContent = 'Disconnected';
  byId('replay-stream').textContent = stream;
  byId('replay-age').textContent = Number.isFinite(age) && age >= 0 ? `${age.toFixed(1)} s` : 'Unknown';
  byId('replay-note').textContent = reason;
  replayList('replay-tags', [], 'Unavailable');
  replayList('replay-candidates', [], 'Unavailable');
}
let replayPending = false;
async function refreshReplay() {
  if (replayPending) return;
  replayPending = true;
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 900);
  try {
    const response = await fetch('http://127.0.0.1:8082/api/scene', {
      cache: 'no-store', signal: controller.signal
    });
    if (!response.ok) throw new Error(`Scene API returned HTTP ${response.status}`);
    const data = await response.json();
    if (!data || typeof data !== 'object') throw new Error('Invalid scene response');
    const stream = typeof data.stream_state === 'string' ? data.stream_state : 'Unknown';
    const age = data.age_since_bridge_receive_s;
    const scene = data.scene;
    const fresh = Number.isFinite(age) && age >= 0 &&
      Number.isFinite(data.stale_after_s) && age <= data.stale_after_s;
    if (stream !== 'receiving' || !scene || scene.status !== 'processed' || data.error || !fresh) {
      disconnectReplay(typeof data.error === 'string' ? data.error : 'No current replay scene. Historical detections are unavailable.', stream, age);
      return;
    }
    if (data.data_kind !== 'historical_replay' || data.execution_allowed !== false ||
        scene.historical_image !== true || scene.execution_allowed !== false ||
        !Array.isArray(scene.robot_targets) || scene.robot_targets.length ||
        !Array.isArray(scene.destination_tags) || !Array.isArray(scene.source_candidates)) {
      throw new Error('Invalid historical replay response');
    }
    const tags = scene.destination_tags.map(tag => {
      if (!tag || !Number.isInteger(tag.tag_id)) throw new Error('Invalid destination tag');
      return `Tag ${tag.tag_id}`;
    });
    const candidates = scene.source_candidates.map(candidate => {
      if (!candidate || typeof candidate.candidate_id !== 'string' || typeof candidate.colour !== 'string') {
        throw new Error('Invalid source candidate');
      }
      return `${candidate.candidate_id} · Colour: ${candidate.colour} · Source variant: unverified`;
    });
    byId('replay-connection').textContent = 'Connected';
    byId('replay-stream').textContent = stream;
    byId('replay-age').textContent = `${age.toFixed(1)} s`;
    byId('replay-note').textContent = 'Historical image capture age is unknown. Source variants are not inferred from destination tags.';
    replayList('replay-tags', tags, 'No destination tags detected');
    replayList('replay-candidates', candidates, 'No source candidates detected');
  } catch (error) {
    disconnectReplay(error.name === 'AbortError' ? 'Scene request timed out.' : `Scene unavailable: ${error.message}`);
  } finally {
    clearTimeout(timeout);
    replayPending = false;
  }
}
refreshReplay();
setInterval(refreshReplay, 1000);

async function poll() {
  await refresh();
  setTimeout(poll, 500);
}
poll();
</script>
</html>
""".replace("__SHARED_STYLE__", STYLE)


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
