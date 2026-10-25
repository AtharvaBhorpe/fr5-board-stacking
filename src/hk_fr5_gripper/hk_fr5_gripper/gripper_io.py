"""Tool I/O access for the gripper: real robot (XML-RPC) or fake (in memory). Decision-log 5l.

Both classes have the same methods:
    set_do(index, on)   switch a tool digital output
    get_di(index)       -> bool, read a tool digital input
    get_do(index)       -> bool, read back a tool digital output
Errors: CommError (controller unreachable / timeout / unexpected reply), ControllerError (error code).
"""
import http.client
import socket
import time
import xml.parsers.expat
import xmlrpc.client


class CommError(Exception):
    """XML-RPC unreachable, timed out, or the reply has an unexpected format."""


class ControllerError(Exception):
    """The robot controller answered with a non-zero error code."""

    def __init__(self, call, code):
        super().__init__(f"{call} returned controller error code {code}")
        self.code = code


class _TimeoutTransport(xmlrpc.client.Transport):
    """xmlrpc Transport with a socket timeout (the default one can block forever)."""

    def __init__(self, timeout):
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = self._timeout
        return conn


def parse_reply(call, reply):
    """Value of a Fairino getter reply. Expected [error, value]; a bare int is taken as the value.

    NOTE: the XML-RPC reply format of GetToolDI / GetToolDO is not verified on the real
    controller yet (step 20f); anything else raises CommError with the raw reply.
    """
    if isinstance(reply, (list, tuple)) and len(reply) >= 2:
        if reply[0] != 0:
            raise ControllerError(call, reply[0])
        reply = reply[1]
    if isinstance(reply, bool) or not isinstance(reply, int):
        raise CommError(f"{call}: unexpected reply {reply!r}")
    return reply


class XmlRpcIo:
    """Real tool I/O via the controller's XML-RPC server (http://<ip>:<port>/RPC2)."""

    def __init__(self, ip, port, timeout):
        self.url = f"http://{ip}:{port}/RPC2"
        self._proxy = xmlrpc.client.ServerProxy(self.url, transport=_TimeoutTransport(timeout))

    def _call(self, name, *args):
        try:
            return getattr(self._proxy, name)(*args)
        except (OSError, socket.timeout, http.client.HTTPException, xmlrpc.client.Error,
                xml.parsers.expat.ExpatError) as e:
            raise CommError(f"{name}{args} via {self.url}: {e}") from None

    def set_do(self, index, on):
        # SetToolDO(id, status, smooth 0 = no, block 0 = blocking)
        code = self._call("SetToolDO", index, 1 if on else 0, 0, 0)
        if code != 0:
            raise ControllerError(f"SetToolDO({index}, {int(on)})", code)

    def get_di(self, index):
        # GetToolDI(id, block 0 = blocking): tool inputs (GetDI would read the control box inputs)
        return parse_reply("GetToolDI", self._call("GetToolDI", index, 0)) == 1

    def get_do(self, index):
        # GetToolDO() -> bit mask of the tool outputs (bit 0 = DO0)
        return bool(parse_reply("GetToolDO", self._call("GetToolDO")) >> index & 1)


class FakeIo:
    """In-memory tool I/O. The vacuum DI goes to 1 `delay` s after the vacuum DO is switched on,
    if vacuum_ok() is true (callable, so a ROS parameter can change it at runtime)."""

    def __init__(self, vacuum_do, vacuum_di, delay, vacuum_ok):
        self._vacuum_do, self._vacuum_di = vacuum_do, vacuum_di
        self._delay, self._vacuum_ok = delay, vacuum_ok
        self._do = {}
        self._vacuum_on_since = None

    def set_do(self, index, on):
        if index == self._vacuum_do:
            if on and not self._do.get(index, False):
                self._vacuum_on_since = time.monotonic()
            elif not on:
                self._vacuum_on_since = None
        self._do[index] = bool(on)

    def get_di(self, index):
        if index != self._vacuum_di or self._vacuum_on_since is None:
            return False
        return self._vacuum_ok() and time.monotonic() - self._vacuum_on_since >= self._delay

    def get_do(self, index):
        return self._do.get(index, False)


class Debounce:
    """A bool that changes only after `reads` equal raw reads in a row (W-8). First read is taken as is."""

    def __init__(self, reads):
        self._reads = reads
        self.value = None  # None until the first read
        self._candidate, self._count = None, 0

    def reset(self):
        """Forget the value (e.g. after a lost connection): the next read is taken as is."""
        self.value, self._candidate, self._count = None, None, 0

    def update(self, raw):
        """Feed one raw read; returns True if the debounced value changed."""
        if self.value is None:
            self.value = raw
            return True
        if raw == self.value:
            self._candidate, self._count = None, 0
            return False
        if raw != self._candidate:
            self._candidate, self._count = raw, 0
        self._count += 1
        if self._count >= self._reads:
            self.value, self._candidate, self._count = raw, None, 0
            return True
        return False
