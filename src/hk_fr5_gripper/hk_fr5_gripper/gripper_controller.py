"""gripper_controller: Schmalz vacuum gripper on the robot tool I/O. Decisions: decision-log 5l (W-*).

    ros2 run hk_fr5_gripper gripper_controller --ros-args --params-file <hk_fr5_gripper>/config/gripper.yaml

Status: state topic, start / shutdown behaviour (step 20b). Grip / release actions: step 20c.
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
import time

import rclpy
from hk_fr5_interfaces.msg import GripperState
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions

from hk_fr5_gripper.gripper_io import CommError, ControllerError, Debounce, FakeIo, XmlRpcIo

S = GripperState
ERROR_STATES = (S.PART_LOST, S.COMM_ERROR, S.CONTROLLER_ERROR)

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
