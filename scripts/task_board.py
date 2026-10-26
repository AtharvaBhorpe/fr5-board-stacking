from web_style import STYLE

import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "docs/FR5_EXECUTION_CHECKLIST.md"
HOST, PORT = "127.0.0.1", 8081
ORIGIN = f"http://{HOST}:{PORT}"
LOCK = threading.Lock()
TASK = re.compile(r"^- \[([ xX])\] (.*)$")

HTML = r"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FR5 Execution Board</title>
<style>__SHARED_STYLE__
#board{display:flex;gap:16px;padding:8px 32px 24px;align-items:flex-start;overflow-x:auto}.column{flex:1 0 250px;min-width:0;background:#e4e9e5;padding:16px;border-radius:6px}.column{max-height:calc(100vh - 330px);overflow-y:auto}.column h2{position:sticky;top:-16px;background:#e4e9e5;padding:8px 0;z-index:1;font-size:16px;display:flex;justify-content:space-between;align-items:center;min-height:48px;gap:8px}.count{background:#fff;padding:2px 9px;border-radius:4px;font-size:14px}.card{background:var(--paper);padding:20px;margin:12px 0;border-radius:4px}.card.done{background:#f5f8f5}.task-meta{color:var(--accent);font-size:14px;font-weight:600;margin-bottom:10px}.title{line-height:1.5;font-weight:600;overflow-wrap:anywhere}.task-detail{font-size:14px;color:var(--muted);margin-top:14px;overflow-wrap:anywhere}.card select{margin-top:20px;width:100%;font-size:14px;background:#f3f6f3;border:0}.empty{padding:30px 4px;color:var(--muted);font-size:14px}.board-tools{display:flex;justify-content:space-between;align-items:center;gap:20px;padding:0 32px 20px}.summary{font-size:15px;color:var(--muted)}@media(max-width:700px){#board{display:block;padding:0 20px}.column{margin-bottom:20px;max-height:none;overflow:visible}.board-tools{padding:0 20px 20px;align-items:start;flex-direction:column}.column h2{min-height:0}}</style>
<body>
<nav class="topbar" aria-label="Workspace"><div class="brand">FR5 / Workspace</div><span>Execution board</span></nav>
<header class="intro"><div><h1>From plan to proof.</h1><p>Keep the next task clear. Track demonstrated results and the work still waiting for the lab.</p></div><div class="notice">Physical validation pending<br>One active implementation task</div></header>
<div class="board-tools"><div><div id="totals" class="summary">Loading checklist…</div><div id="message" role="status" aria-live="polite"></div></div><button class="secondary" id="refresh">Refresh checklist</button></div>
<main id="board" aria-label="Task columns"></main>
<footer>Move a task with its dropdown. Changes save to the repository checklist. Record evidence before marking work done.</footer>
<noscript>This board requires JavaScript. You can still edit docs/FR5_EXECUTION_CHECKLIST.md directly.</noscript>
<script>
let state, busy=false;
const message=document.getElementById("message");
async function request(url, options){
  const response=await fetch(url,options);
  const data=await response.json();
  if(!response.ok) throw new Error(data.error || "Request failed");
  return data;
}
function render(){
  const board=document.getElementById("board");
  board.replaceChildren();
  const done=state.cards.filter(c=>c.done).length;
  document.getElementById("totals").textContent=`${state.cards.length} tasks · ${done} done · ${state.cards.length-done} remaining`;
  for(const column of state.columns){
    const section=document.createElement("section");
    section.className="column";
    const cards=state.cards.filter(c=>c.column===column);
    const heading=document.createElement("h2");
    heading.textContent=column.replace(" — evidence scope matters", "");
    const count=document.createElement("span"); count.className="count"; count.textContent=cards.length; heading.append(count);
    section.append(heading);
    if(!cards.length){
      const empty=document.createElement("div");
      empty.className="empty"; empty.textContent="No tasks";
      section.append(empty);
    }
    for(const card of cards){
      const item=document.createElement("article");
      item.className="card"+(card.done?" done":"");
      const title=document.createElement("div");
      title.className="title";
      const parts=card.text.match(/^(T\d+|OPTIONAL) — (?:(Person [AB]|Both): )?(.*)$/);
      const text=parts?parts[3]:card.text;
      if(parts){const meta=document.createElement("div");meta.className="task-meta";meta.textContent=parts[1]+(parts[2]?" · "+parts[2]:"");item.append(meta);}
      const split=text.search(/ (?:Acceptance:|Requires |Commit:|Commits:|Evidence:)/);
      title.textContent=split<0?text:text.slice(0,split);
      item.append(title);
      if(split>=0){const detail=document.createElement("div");detail.className="task-detail";detail.textContent=text.slice(split).trim();item.append(detail);}
      const select=document.createElement("select");
      select.setAttribute("aria-label","Move "+card.text);
      const placeholder=new Option("Move to…","");
      placeholder.disabled=true; placeholder.selected=true;
      select.add(placeholder);
      for(const target of state.columns){
        if(target!==column) select.add(new Option(target,target));
      }
      select.onchange=()=>move(card.id,select.value);
      item.append(select); section.append(item);
    }
    board.append(section);
  }
}
async function refresh(){
  if(busy)return;
  busy=true;
  try{
    state=await request("/api/board");
    render(); message.textContent="Loaded current checklist.";
  }catch(error){message.textContent=error.message;}
  finally{busy=false;}
}
async function move(id,column){
  if(busy)return;
  busy=true;
  document.querySelectorAll("select,button").forEach(e=>e.disabled=true);
  try{
    state=await request("/api/move",{
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({id,column,revision:state.revision})
    });
    render(); message.textContent="Saved to checklist → "+column;
  }catch(error){
    message.textContent=error.message+" Click Refresh before trying again.";
  }finally{
    busy=false;
    document.querySelectorAll("select,button").forEach(e=>e.disabled=false);
  }
}
document.getElementById("refresh").onclick=refresh;
refresh();
</script>
</html>""".replace("__SHARED_STYLE__", STYLE)

def snapshot():
    raw = BOARD.read_bytes()
    lines = raw.decode("utf-8").splitlines()
    columns, cards = [], []
    column = None
    for index, line in enumerate(lines):
        if line.startswith("## "):
            column = line[3:].strip()
            if column in columns:
                raise ValueError("Duplicate column headings in checklist")
            columns.append(column)
        match = TASK.match(line)
        if match and column is not None:
            cards.append({
                "id": index,
                "column": column,
                "text": match[2],
                "done": match[1].lower() == "x",
            })
    return lines, {
        "revision": hashlib.sha256(raw).hexdigest(),
        "columns": columns,
        "cards": cards,
    }

def save(lines):
    original_mode = BOARD.stat().st_mode & 0o777
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=BOARD.parent,
            prefix=".fr5-board-", delete=False
        ) as handle:
            temporary = Path(handle.name)
            handle.write("\n".join(lines) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.chmod(original_mode)
        os.replace(temporary, BOARD)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)

class Handler(BaseHTTPRequestHandler):
    def reply(self, status, data, content_type="application/json"):
        payload = (
            json.dumps(data).encode()
            if content_type == "application/json" else data.encode()
        )
        self.send_response(status)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def valid_host(self):
        return self.headers.get("Host") == f"{HOST}:{PORT}"

    def do_GET(self):
        if not self.valid_host():
            return self.reply(403, {"error": "Use " + ORIGIN})
        try:
            if self.path == "/":
                return self.reply(200, HTML, "text/html")
            if self.path == "/api/board":
                with LOCK:
                    _, state = snapshot()
                return self.reply(200, state)
            self.reply(404, {"error": "Not found"})
        except Exception as error:
            self.reply(500, {"error": str(error)})

    def do_POST(self):
        if (
            not self.valid_host()
            or self.headers.get("Origin") != ORIGIN
        ):
            return self.reply(403, {"error": "Open the board at " + ORIGIN})
        if self.path != "/api/move":
            return self.reply(404, {"error": "Not found"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(length))
            with LOCK:
                lines, state = snapshot()
                if data.get("revision") != state["revision"]:
                    return self.reply(409, {
                        "error": "Checklist changed since this page was loaded."
                    })
                target = data.get("column")
                card = next(
                    (c for c in state["cards"] if c["id"] == data.get("id")),
                    None,
                )
                if card is None or target not in state["columns"]:
                    raise ValueError("Unknown task or column")
                if target != card["column"]:
                    lines.pop(card["id"])
                    start = next(
                        i for i, line in enumerate(lines)
                        if line.startswith("## ") and line[3:].strip() == target
                    )
                    end = next(
                        (i for i in range(start + 1, len(lines))
                         if lines[i].startswith("## ")),
                        len(lines),
                    )
                    while end > start + 1 and not lines[end - 1].strip():
                        end -= 1
                    checked = "x" if target.lower().startswith("done") else " "
                    lines.insert(end, f"- [{checked}] {card['text']}")
                    save(lines)
                _, updated = snapshot()
            self.reply(200, updated)
        except (ValueError, KeyError, TypeError) as error:
            self.reply(400, {"error": str(error)})
        except Exception as error:
            self.reply(500, {"error": str(error)})

    def log_message(self, *_):
        pass

if __name__ == "__main__":
    if not BOARD.is_file():
        raise SystemExit(f"Checklist not found: {BOARD}")
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"FR5 task board: {ORIGIN}", flush=True)
    print(f"Saving to: {BOARD}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
