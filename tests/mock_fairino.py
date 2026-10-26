"""Mock Fairino XML-RPC server (tool DO/DI only) for tests. Reply format = our assumption [err, value].
Run: python3 mock_fairino.py PORT [do_mask] [di0]. Extra RPCs for tests: _set_di(i, v), _log(), _set_delay(s), _set_err(code)."""
import sys, threading, time
from xmlrpc.server import SimpleXMLRPCServer
port = int(sys.argv[1]); do = int(sys.argv[2]) if len(sys.argv) > 2 else 0
di = [int(sys.argv[3]) if len(sys.argv) > 3 else 0, 0]
log, cfg = [], {"delay": 0.0, "err": 0, "auto_vac": False}
def SetToolDO(i, s, smooth, block):
    global do
    time.sleep(cfg["delay"]); log.append(("SetToolDO", i, s))
    if cfg["err"]: return cfg["err"]
    do = (do | (1 << i)) if s else (do & ~(1 << i))
    if cfg["auto_vac"] and i == 0: di[0] = s
    return 0
def GetToolDI(i, block):
    time.sleep(cfg["delay"])
    return [cfg["err"], di[i] if not cfg["err"] else 0]
def GetToolDO():
    time.sleep(cfg["delay"]); return [cfg["err"], do]
srv = SimpleXMLRPCServer(("127.0.0.1", port), logRequests=False, allow_none=True)
for f in (SetToolDO, GetToolDI, GetToolDO): srv.register_function(f)
srv.register_function(lambda i, v: di.__setitem__(i, v) or 0, "_set_di")
srv.register_function(lambda: log, "_log")
srv.register_function(lambda s: cfg.__setitem__("delay", s) or 0, "_set_delay")
srv.register_function(lambda c: cfg.__setitem__("err", c) or 0, "_set_err")
srv.register_function(lambda b: cfg.__setitem__("auto_vac", b) or 0, "_auto_vac")
print(f"mock fairino on {port}", flush=True)
srv.serve_forever()
