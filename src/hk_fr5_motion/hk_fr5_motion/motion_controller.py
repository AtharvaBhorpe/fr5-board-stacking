"""motion_controller: /motion/move_to, /motion/move_delta (actions), /motion/get_state (service).

    ros2 run hk_fr5_motion motion_controller --ros-args --params-file <hk_fr5_motion>/config/motion.yaml

Step 14 skeleton: interfaces advertised, config loaded, every goal validated; no motion yet
(valid goals end with FAILURE "not implemented yet"). Decisions: decision-log 5f (C-1..C-7).
- Single-threaded executor + async callbacks (await MoveIt without blocking the node).
- One active goal for both actions: a new goal preempts the running one (PREEMPTED).
- Invalid goals are accepted, then aborted with INVALID_GOAL_CONSTRAINTS and a clear message.
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from hk_fr5_interfaces.action import MoveDelta, MoveTo
from hk_fr5_interfaces.srv import GetState
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

E = MoveItErrorCodes
PLANNERS = ("", "PTP", "LIN")  # "" = OMPL RRTConnect, else Pilz (F6)

# name -> type; values come only from config/motion.yaml (no defaults in code)
PARAMS = {
    "group": Parameter.Type.STRING,
    "tcp_link": Parameter.Type.STRING,
    "base_frame": Parameter.Type.STRING,
    "move_group_action": Parameter.Type.STRING,
    "default_velocity_scaling": Parameter.Type.DOUBLE,
    "planning_time": Parameter.Type.DOUBLE,
    "planning_attempts": Parameter.Type.INTEGER,
    "replan_retries": Parameter.Type.INTEGER,
    "position_tolerance": Parameter.Type.DOUBLE,
    "orientation_tolerance": Parameter.Type.DOUBLE,
    "server_timeout": Parameter.Type.DOUBLE,
    "state_timeout": Parameter.Type.DOUBLE,
}


def _bad_number(goal, fields):
    """Name of the first field that is NaN or infinite, else None."""
    for f in fields:
        if not math.isfinite(getattr(goal, f)):
            return f
    return None


def validate_move_to(goal, named_targets):
    """(error_code, message) if the goal is invalid, else None."""
    bad = _bad_number(goal, ("x", "y", "z", "roll", "pitch", "yaw", "velocity_scaling"))
    if bad:
        return E.INVALID_GOAL_CONSTRAINTS, f"'{bad}' is not a finite number"
    if not 0.0 <= goal.velocity_scaling <= 1.0:
        return E.INVALID_GOAL_CONSTRAINTS, f"velocity_scaling must be in [0, 1], got {goal.velocity_scaling}"
    if goal.mode not in (MoveTo.Goal.MODE_DOWN_YAW, MoveTo.Goal.MODE_FULL):
        return E.INVALID_GOAL_CONSTRAINTS, f"mode must be 0 (MODE_DOWN_YAW) or 1 (MODE_FULL), got {goal.mode}"
    if goal.planner not in PLANNERS:
        return E.INVALID_GOAL_CONSTRAINTS, f"planner must be '', 'PTP' or 'LIN', got '{goal.planner}'"
    if goal.named_target:
        if named_targets is None:
            return E.COMMUNICATION_FAILURE, "no SRDF yet on /robot_description_semantic (is move_group running?)"
        if goal.named_target not in named_targets:
            return E.INVALID_GOAL_CONSTRAINTS, (f"unknown named_target '{goal.named_target}', "
                                                f"known: {sorted(named_targets)}")
    return None


def validate_move_delta(goal):
    """(error_code, message) if the goal is invalid, else None."""
    bad = _bad_number(goal, ("dx", "dy", "dz", "droll", "dpitch", "dyaw", "velocity_scaling"))
    if bad:
        return E.INVALID_GOAL_CONSTRAINTS, f"'{bad}' is not a finite number"
    if not 0.0 <= goal.velocity_scaling <= 1.0:
        return E.INVALID_GOAL_CONSTRAINTS, f"velocity_scaling must be in [0, 1], got {goal.velocity_scaling}"
    if goal.frame not in (MoveDelta.Goal.FRAME_TCP, MoveDelta.Goal.FRAME_BASE):
        return E.INVALID_GOAL_CONSTRAINTS, f"frame must be 0 (FRAME_TCP) or 1 (FRAME_BASE), got {goal.frame}"
    return None


def group_states(srdf_xml, group):
    """{state name: {joint: value}} for one SRDF group."""
    states = {}
    for gs in ET.fromstring(srdf_xml).iter("group_state"):
        if gs.get("group") == group:
            states[gs.get("name")] = {j.get("name"): float(j.get("value")) for j in gs.iter("joint")}
    return states


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller", namespace="motion")
        self.cfg = {}
        for name, ptype in PARAMS.items():
            self.declare_parameter(name, ptype)
            try:
                self.cfg[name] = self.get_parameter(name).value
            except ParameterUninitializedException:
                raise RuntimeError(f"parameter '{name}' missing: start with "
                                   "--params-file <hk_fr5_motion share>/config/motion.yaml") from None

        self.named_targets = None  # filled from the SRDF
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description_semantic", self._on_srdf, latched)

        self._active = None  # goal handle of the running goal (both actions)
        self._move_to = ActionServer(self, MoveTo, "move_to", self._execute_move_to,
                                     goal_callback=self._accept, cancel_callback=self._cancel,
                                     handle_accepted_callback=self._start)
        self._move_delta = ActionServer(self, MoveDelta, "move_delta", self._execute_move_delta,
                                        goal_callback=self._accept, cancel_callback=self._cancel,
                                        handle_accepted_callback=self._start)
        self.create_service(GetState, "get_state", self._get_state)
        self.get_logger().info(f"ready: {self.get_namespace()}/move_to, move_delta, get_state | config {self.cfg}")

    # ---- SRDF -------------------------------------------------------------------------------
    def _on_srdf(self, msg):
        self.named_targets = group_states(msg.data, self.cfg["group"])
        self.get_logger().info(f"SRDF received, named targets for '{self.cfg['group']}': "
                               f"{sorted(self.named_targets)}")

    # ---- goal handling (shared by both actions) ----------------------------------------------
    def _accept(self, _goal_request):
        return GoalResponse.ACCEPT  # invalid goals are aborted with a reason in execute (C-3)

    def _cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _start(self, goal_handle):
        if self._active is not None and self._active.is_active:
            self._preempt(self._active)
        self._active = goal_handle
        goal_handle.execute()

    def _preempt(self, old_goal_handle):
        # step 16+: cancel the running MoveIt goal; the old execute callback then ends with PREEMPTED
        self.get_logger().info("new goal received: preempting the running goal")

    def _finish(self, goal_handle, result_type, code, message):
        result = result_type(success=(code == E.SUCCESS), message=message, error_code=E(val=code))
        result.final_tcp_pose.header.frame_id = self.cfg["base_frame"]
        if code == E.SUCCESS:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        if self._active is goal_handle:
            self._active = None
        log = self.get_logger().info if code == E.SUCCESS else self.get_logger().warn
        log(f"{result_type.__module__.split('.')[-1]}: code {code}: {message}")
        return result

    # ---- actions -----------------------------------------------------------------------------
    async def _execute_move_to(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"move_to: named_target='{g.named_target}' xyz=({g.x:.4f}, {g.y:.4f}, {g.z:.4f}) "
            f"rpy=({g.roll:.3f}, {g.pitch:.3f}, {g.yaw:.3f}) mode={g.mode} planner='{g.planner}' "
            f"scaling={g.velocity_scaling} plan_only={g.plan_only}")
        error = validate_move_to(g, self.named_targets)
        if error:
            return self._finish(goal_handle, MoveTo.Result, *error)
        return self._finish(goal_handle, MoveTo.Result, E.FAILURE, "move_to not implemented yet (step 16)")

    async def _execute_move_delta(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"move_delta: d=({g.dx:.4f}, {g.dy:.4f}, {g.dz:.4f}) drpy=({g.droll:.3f}, {g.dpitch:.3f}, "
            f"{g.dyaw:.3f}) frame={g.frame} scaling={g.velocity_scaling} plan_only={g.plan_only}")
        error = validate_move_delta(g)
        if error:
            return self._finish(goal_handle, MoveDelta.Result, *error)
        return self._finish(goal_handle, MoveDelta.Result, E.FAILURE, "move_delta not implemented yet (step 17)")

    # ---- service -----------------------------------------------------------------------------
    def _get_state(self, _request, response):
        response.success = False
        response.message = "get_state not implemented yet (step 15)"
        return response


def main():
    rclpy.init()
    try:
        node = MotionController()
    except RuntimeError as e:
        print(f"[motion_controller] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)  # single-threaded executor; async callbacks interleave (C-1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
