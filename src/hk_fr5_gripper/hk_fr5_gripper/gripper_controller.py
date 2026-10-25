"""gripper_controller: Schmalz vacuum gripper on the robot tool I/O. Decisions: decision-log 5l (W-*).

    ros2 run hk_fr5_gripper gripper_controller --ros-args --params-file <hk_fr5_gripper>/config/gripper.yaml

- /gripper/grip (Grip): blow-off off, vacuum pump on, wait up to timeout for vacuum achieved;
  success -> pump stays on (HOLDING); timeout -> pump off, NO_VACUUM; cancel -> pump off, CANCELLED (W-3);
  already HOLDING with vacuum -> SUCCESS at once, nothing switched (W-5).
- /gripper/release (Release): vacuum pump off, blow-off on for blow_off_time, blow-off off (W-6).
- One active goal for both actions: a new goal preempts the running one (PREEMPTED, W-7). The preempted
  goal switches nothing; the new goal takes over the outputs. Invalid goals -> INVALID_GOAL.
- I/O error inside a goal -> both pumps off (best effort), COMM_FAILURE / CONTROLLER_ERROR.
- /gripper/state (GripperState, latched): DO / DI values + state, at state_rate and on every change.
- I/O: vacuum pump = vacuum_do, blow-off pump = blow_off_do, vacuum achieved = vacuum_di.
  Real robot via XML-RPC (SetToolDO / GetToolDI / GetToolDO), fake mode in memory.
- Start: reads the I/O and publishes it, switches nothing (W-9).
- Shutdown: vacuum pump and blow-off pump off (W-10).
- DI read every poll_period, debounced (W-8). HOLDING and vacuum lost -> PART_LOST, pump stays on (W-4).
- I/O errors -> COMM_ERROR / CONTROLLER_ERROR (W-12). COMM_ERROR clears by itself on the next good read
  (W-13); PART_LOST / CONTROLLER_ERROR are cleared by a new grip / release.
- Single-threaded executor; each I/O call is bounded by rpc_timeout.
"""
import math
import time

import rclpy
from hk_fr5_interfaces.action import Grip, Release
from hk_fr5_interfaces.msg import GripperErrorCode, GripperState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future

from hk_fr5_gripper.gripper_io import CommError, ControllerError, Debounce, FakeIo, XmlRpcIo

S = GripperState
C = GripperErrorCode
ERROR_STATES = (S.PART_LOST, S.COMM_ERROR, S.CONTROLLER_ERROR)
CODE_NAMES = {v: k for k, v in vars(C).items() if k.isupper() and isinstance(v, int)}

# name -> type; values come only from config/gripper.yaml (no defaults in code)
PARAMS = {
    "use_fake_hardware": Parameter.Type.BOOL,
    "robot_ip": Parameter.Type.STRING,
    "xmlrpc_port": Parameter.Type.INTEGER,
    "rpc_timeout": Parameter.Type.DOUBLE,
    "comm_retry_period": Parameter.Type.DOUBLE,
    "vacuum_do": Parameter.Type.INTEGER,
    "blow_off_do": Parameter.Type.INTEGER,
    "vacuum_di": Parameter.Type.INTEGER,
    "grip_timeout": Parameter.Type.DOUBLE,
    "blow_off_time": Parameter.Type.DOUBLE,
    "poll_period": Parameter.Type.DOUBLE,
    "debounce_reads": Parameter.Type.INTEGER,
    "state_rate": Parameter.Type.DOUBLE,
    "fake_vacuum": Parameter.Type.BOOL,
    "fake_vacuum_delay": Parameter.Type.DOUBLE,
}
POSITIVE = ("rpc_timeout", "comm_retry_period", "grip_timeout", "blow_off_time", "poll_period",
            "state_rate")
TOOL_IO = ("vacuum_do", "blow_off_do", "vacuum_di")  # FR5 tool I/O: index 0 or 1


def validate_time(value, name):
    """Error message for a bad goal time (s, 0 = config default), else None."""
    if not math.isfinite(value) or value < 0.0:
        return f"'{name}' must be a finite number >= 0 (0 = default), got {value}"
    return None


def validate_config(cfg):
    """Error message for a bad config, else None."""
    for name in POSITIVE:
        if not cfg[name] > 0.0:
            return f"'{name}' must be > 0, got {cfg[name]}"
    for name in TOOL_IO:
        if cfg[name] not in (0, 1):
            return f"'{name}' must be 0 or 1 (FR5 tool I/O), got {cfg[name]}"
    if cfg["vacuum_do"] == cfg["blow_off_do"]:
        return "'vacuum_do' and 'blow_off_do' must differ"
    if cfg["debounce_reads"] < 1:
        return f"'debounce_reads' must be >= 1, got {cfg['debounce_reads']}"
    if cfg["fake_vacuum_delay"] < 0.0:
        return f"'fake_vacuum_delay' must be >= 0, got {cfg['fake_vacuum_delay']}"
    return None


class GripperController(Node):
    def __init__(self):
        super().__init__("gripper_controller", namespace="gripper")
        self.cfg = {}
        for name, ptype in PARAMS.items():
            self.declare_parameter(name, ptype)
            try:
                self.cfg[name] = self.get_parameter(name).value
            except ParameterUninitializedException:
                raise RuntimeError(f"parameter '{name}' missing: start with "
                                   "--params-file <hk_fr5_gripper share>/config/gripper.yaml") from None
        error = validate_config(self.cfg)
        if error:
            raise RuntimeError(f"bad config: {error}")
        c = self.cfg
        self.vacuum_do, self.blow_off_do, self.vacuum_di = c["vacuum_do"], c["blow_off_do"], c["vacuum_di"]

        if c["use_fake_hardware"]:
            self.io = FakeIo(self.vacuum_do, self.vacuum_di, c["fake_vacuum_delay"],
                             lambda: self.get_parameter("fake_vacuum").value)
            self.get_logger().info("FAKE gripper I/O (in memory, no robot connection)")
        else:
            self.io = XmlRpcIo(c["robot_ip"], c["xmlrpc_port"], c["rpc_timeout"])
            self.get_logger().warn(f"REAL gripper I/O via XML-RPC {self.io.url} "
                                   f"(vacuum DO{self.vacuum_do}, blow-off DO{self.blow_off_do}, "
                                   f"vacuum achieved DI{self.vacuum_di})")

        self.outputs = {self.vacuum_do: False, self.blow_off_do: False}  # last known DO states
        self.vacuum = Debounce(c["debounce_reads"])                    # debounced vacuum achieved
        self.state, self.message = S.IDLE, ""
        self._poll_after = 0.0  # monotonic time; > now while backing off after a comm failure
        self._started = False   # start read done (W-9); until then a recovery repeats it
        self._before_comm_error = S.IDLE  # state to return to when communication works again (W-13)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(GripperState, "state", latched)
        self._read_start()
        self.create_timer(c["poll_period"], self._poll)
        self.create_timer(1.0 / c["state_rate"], self.publish_state)

        self._active = None  # goal handle of the running goal (both actions)
        self._grip = ActionServer(self, Grip, "grip", self._execute_grip,
                                  goal_callback=self._accept, cancel_callback=self._cancel,
                                  handle_accepted_callback=self._start)
        self._release = ActionServer(self, Release, "release", self._execute_release,
                                     goal_callback=self._accept, cancel_callback=self._cancel,
                                     handle_accepted_callback=self._start)

    # ---- state -------------------------------------------------------------------------------
    def publish_state(self):
        msg = GripperState(state=self.state, message=self.message,
                           vacuum_on=self.outputs[self.vacuum_do],
                           blow_off_on=self.outputs[self.blow_off_do],
                           vacuum_achieved=bool(self.vacuum.value))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def set_state(self, state, message):
        """Change state + message, log it, publish at once."""
        # separate calls: rclpy fixes the severity per call site (one shared call raises ValueError)
        if state in ERROR_STATES:
            self.get_logger().warn(f"{state}: {message}")
        else:
            self.get_logger().info(f"{state}: {message}")
        self.state, self.message = state, message
        self.publish_state()

    def io_error(self, e):
        """Map an I/O exception to COMM_ERROR / CONTROLLER_ERROR (W-12). The same error again is not re-logged."""
        if isinstance(e, CommError):
            self._poll_after = time.monotonic() + self.cfg["comm_retry_period"]  # back off
        state = S.COMM_ERROR if isinstance(e, CommError) else S.CONTROLLER_ERROR
        if state == S.COMM_ERROR and self.state != S.COMM_ERROR:
            self._before_comm_error = self.state
            self.vacuum.reset()  # DI unknown during the outage; first read after it is taken as is
        if (state, str(e)) != (self.state, self.message):
            self.set_state(state, str(e))

    # ---- I/O ---------------------------------------------------------------------------------
    def _read_start(self):
        """Read the outputs and the vacuum input once; switch nothing (W-9)."""
        try:
            for index in self.outputs:
                self.outputs[index] = self.io.get_do(index)
            self.vacuum.update(self.io.get_di(self.vacuum_di))
        except (CommError, ControllerError) as e:
            self.io_error(e)
            return
        vac, blow, held = self.outputs[self.vacuum_do], self.outputs[self.blow_off_do], self.vacuum.value
        detail = (f"start: vacuum pump {'on' if vac else 'off'}, blow-off {'on' if blow else 'off'}, "
                  f"vacuum {'achieved' if held else 'not achieved'} (no output switched)")
        self._started = True
        self.set_state(S.HOLDING if vac and held else S.IDLE, detail)

    def _recover(self):
        """First good read after COMM_ERROR (W-13): re-read the outputs (controller may have restarted),
        go back to the state before the error; HOLDING without vacuum -> PART_LOST."""
        if not self._started:
            self._read_start()
            return
        try:
            for index in self.outputs:
                self.outputs[index] = self.io.get_do(index)
        except (CommError, ControllerError) as e:
            self.io_error(e)
            return
        state, held = self._before_comm_error, self.vacuum.value
        vac = self.outputs[self.vacuum_do]
        detail = (f"communication restored: vacuum pump {'on' if vac else 'off'}, "
                  f"vacuum {'achieved' if held else 'not achieved'}")
        if state in (S.GRIPPING, S.RELEASING) and not self._goal_running():
            state = S.HOLDING if vac and held else S.IDLE  # that goal has ended meanwhile
        if state == S.HOLDING and not (vac and held):
            state, detail = S.PART_LOST, detail + " -> vacuum lost while holding"
        self.set_state(state, detail)

    def _poll(self):
        """Read the vacuum DI, debounce it, detect a lost part while HOLDING (W-4)."""
        if time.monotonic() < self._poll_after:
            return  # backing off after a comm failure
        try:
            changed = self.vacuum.update(self.io.get_di(self.vacuum_di))
        except (CommError, ControllerError) as e:
            self.io_error(e)
            return
        if self.state == S.COMM_ERROR:
            self._recover()  # clears COMM_ERROR by itself (W-13)
            return
        if not changed:
            return
        if self.state == S.HOLDING and not self.vacuum.value:
            self.set_state(S.PART_LOST, "vacuum lost while holding (vacuum pump left on)")
        else:
            self.get_logger().info(f"vacuum {'achieved' if self.vacuum.value else 'not achieved'}")
            self.publish_state()

    def switch(self, index, on):
        """Switch one tool DO and remember it. Raises CommError / ControllerError."""
        self.io.set_do(index, on)
        self.outputs[index] = on

    def outputs_off(self):
        """Vacuum pump and blow-off pump off (shutdown, W-10). Tries both even if one fails."""
        for index, name in ((self.vacuum_do, "vacuum pump"), (self.blow_off_do, "blow-off pump")):
            try:
                self.switch(index, False)
                self.get_logger().info(f"shutdown: {name} (DO{index}) off")
            except (CommError, ControllerError) as e:
                self.get_logger().error(f"shutdown: {name} (DO{index}) NOT switched off: {e}")


    # ---- goal handling (shared by both actions) ----------------------------------------------
    def _accept(self, _goal_request):
        return GoalResponse.ACCEPT  # invalid goals are aborted with a reason in execute

    def _cancel(self, _goal_handle):
        self.get_logger().info("cancel requested")
        return CancelResponse.ACCEPT

    def _start(self, goal_handle):
        if self._goal_running():
            self.get_logger().info("new goal received: preempting the running goal")
        self._active = goal_handle  # the old goal sees this and ends PREEMPTED (W-7)
        goal_handle.execute()

    def _goal_running(self):
        return self._active is not None and self._active.is_active

    def _finish(self, goal_handle, result_type, code, detail, **fields):
        message = f"{CODE_NAMES.get(code, str(code))}: {detail}"
        result = result_type(success=(code == C.SUCCESS), error_code=C(val=code), message=message, **fields)
        if code == C.SUCCESS:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        if self._active is goal_handle:
            self._active = None
        name = result_type.__name__.split("_")[0].lower()  # Grip_GetResult_Response... -> grip
        if code == C.SUCCESS:  # rclpy fixes the severity per call site: one call per severity
            self.get_logger().info(f"{name}: {message}")
        else:
            self.get_logger().warn(f"{name}: {message}")
        return result

    def _io_abort(self, goal_handle, result_type, e, **fields):
        """I/O error inside a goal: error state, both pumps off (best effort), failure result."""
        self.io_error(e)
        off = []
        for index in (self.vacuum_do, self.blow_off_do):
            try:
                self.switch(index, False)
                off.append(f"DO{index} off")
            except (CommError, ControllerError):
                off.append(f"DO{index} NOT confirmed off")
        code = C.COMM_FAILURE if isinstance(e, CommError) else C.CONTROLLER_ERROR
        return self._finish(goal_handle, result_type, code, f"{e} ({', '.join(off)})", **fields)

    async def _sleep(self, seconds):
        """Non-blocking sleep inside an async callback (timer-backed future)."""
        done = Future()
        timer = self.create_timer(seconds, lambda: done.done() or done.set_result(None))
        await done
        self.destroy_timer(timer)

    # ---- grip ----------------------------------------------------------------------------------
    async def _execute_grip(self, goal_handle):
        r, fail = Grip.Result, {"time_to_vacuum": -1.0}
        error = validate_time(goal_handle.request.timeout, "timeout")
        if error:
            return self._finish(goal_handle, r, C.INVALID_GOAL, error, **fail)
        timeout = goal_handle.request.timeout or self.cfg["grip_timeout"]
        if self.state == S.HOLDING and self.vacuum.value:  # W-5
            return self._finish(goal_handle, r, C.SUCCESS, "already holding (vacuum achieved), nothing switched",
                                time_to_vacuum=0.0)
        self.set_state(S.GRIPPING, f"vacuum pump on, waiting up to {timeout:.1f} s for vacuum")
        try:
            self.switch(self.blow_off_do, False)
            self.switch(self.vacuum_do, True)
        except (CommError, ControllerError) as e:
            return self._io_abort(goal_handle, r, e, **fail)
        self.vacuum.reset(False)  # only fresh reads after pump on count (debounced)
        start = time.monotonic()
        while True:
            await self._sleep(self.cfg["poll_period"])
            elapsed = time.monotonic() - start
            if self._active is not goal_handle:  # preempted: the new goal owns the outputs
                return self._finish(goal_handle, r, C.PREEMPTED, "interrupted by a new gripper goal", **fail)
            if goal_handle.is_cancel_requested:
                return self._pump_off(goal_handle, C.CANCELLED, "cancelled, vacuum pump off")
            if self.vacuum.value:
                self.set_state(S.HOLDING, f"vacuum achieved in {elapsed:.2f} s")
                return self._finish(goal_handle, r, C.SUCCESS, f"vacuum achieved in {elapsed:.2f} s",
                                    time_to_vacuum=elapsed)
            if elapsed >= timeout:
                if self.state == S.COMM_ERROR:  # DI could not be read: say so, not just "no vacuum"
                    return self._io_abort(goal_handle, r, CommError(f"no DI reading: {self.message}"), **fail)
                return self._pump_off(goal_handle, C.NO_VACUUM, f"no vacuum within {timeout:.1f} s, vacuum pump off")
            goal_handle.publish_feedback(Grip.Feedback(elapsed=elapsed))

    def _pump_off(self, goal_handle, code, detail):
        """End a grip with the vacuum pump off (timeout / cancel)."""
        try:
            self.switch(self.vacuum_do, False)
        except (CommError, ControllerError) as e:
            return self._io_abort(goal_handle, Grip.Result, e, time_to_vacuum=-1.0)
        self.set_state(S.IDLE, detail)
        return self._finish(goal_handle, Grip.Result, code, detail, time_to_vacuum=-1.0)

    # ---- release -------------------------------------------------------------------------------
    async def _execute_release(self, goal_handle):
        r = Release.Result
        error = validate_time(goal_handle.request.blow_off_time, "blow_off_time")
        if error:
            return self._finish(goal_handle, r, C.INVALID_GOAL, error)
        blow_time = goal_handle.request.blow_off_time or self.cfg["blow_off_time"]
        self.set_state(S.RELEASING, f"vacuum pump off, blow-off on for {blow_time:.1f} s")
        try:
            self.switch(self.vacuum_do, False)
            self.switch(self.blow_off_do, True)
        except (CommError, ControllerError) as e:
            return self._io_abort(goal_handle, r, e)
        goal_handle.publish_feedback(Release.Feedback(phase="BLOW_OFF"))
        end = time.monotonic() + blow_time
        while (remaining := end - time.monotonic()) > 0.0:
            await self._sleep(min(self.cfg["poll_period"], remaining))
            if self._active is not goal_handle:  # preempted: the new goal owns the outputs
                return self._finish(goal_handle, r, C.PREEMPTED, "interrupted by a new gripper goal")
            if goal_handle.is_cancel_requested:
                break
        try:
            self.switch(self.blow_off_do, False)
        except (CommError, ControllerError) as e:
            return self._io_abort(goal_handle, r, e)
        if goal_handle.is_cancel_requested:
            self.set_state(S.IDLE, "release cancelled, blow-off stopped early")
            return self._finish(goal_handle, r, C.CANCELLED, "cancelled, blow-off stopped early")
        self.set_state(S.IDLE, f"released (blow-off {blow_time:.1f} s)")
        return self._finish(goal_handle, r, C.SUCCESS, f"released (blow-off {blow_time:.1f} s)")


def main():
    # no rclpy signal handler: Ctrl+C raises KeyboardInterrupt while the context is still valid,
    # so the shutdown switch-off below can still log to /rosout (W-10)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = GripperController()
    except RuntimeError as e:
        print(f"[gripper_controller] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)  # single-threaded executor
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C -> KeyboardInterrupt; ExternalShutdownException if shut down elsewhere
    finally:
        node.outputs_off()  # W-10
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
