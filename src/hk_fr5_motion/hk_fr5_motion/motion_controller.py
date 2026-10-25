"""motion_controller: /motion/move_to, /motion/move_delta (actions), /motion/get_state (service).

    ros2 run hk_fr5_motion motion_controller --ros-args --params-file <hk_fr5_motion>/config/motion.yaml

Status: validation (step 14), get_state (step 15), move_to (step 16), move_delta (step 17).
Decisions: decision-log 5f (C-*), 5g (S-*), 5h (T-*), 5i (U-*), 5j (V-*).
- Single-threaded executor + async callbacks (await MoveIt without blocking the node).
- One active goal for both actions: a new goal preempts the running one (PREEMPTED);
  cancel / preempt cancel the running MoveIt goal.
- Invalid goals are accepted, then aborted with INVALID_GOAL_CONSTRAINTS and a clear message.
- move_to: one /move_action call (plan + execute, or plan only); OMPL retried on -1/-2 (T-3).
- move_delta: target from the current tcp pose (TF) + delta, always Pilz LIN, no retry (U-*).
- Cancel / preempt: MoveIt Humble ignores cancel during execution, so a smooth deceleration
  trajectory is sent to the arm controller (stop_deceleration), and the goal ends once stopped (5j).
"""
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseStamped
from hk_fr5_interfaces.action import MoveDelta, MoveTo
from hk_fr5_interfaces.srv import GetState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from shape_msgs.msg import SolidPrimitive
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

E = MoveItErrorCodes
PLANNERS = ("", "PTP", "LIN")  # "" = OMPL RRTConnect, else Pilz (F6)
PIPELINES = {"": ("ompl", "RRTConnect"),  # planner field -> (pipeline_id, planner_id)
             "PTP": ("pilz_industrial_motion_planner", "PTP"),
             "LIN": ("pilz_industrial_motion_planner", "LIN")}
RETRY_CODES = (E.PLANNING_FAILED, E.INVALID_MOTION_PLAN)  # OMPL is random: retry these (T-3)
CODE_NAMES = {v: k for k, v in vars(E).items() if k.isupper() and isinstance(v, int)}
HINTS = {  # short explanations for the common codes (T-6)
    E.SUCCESS: "done",
    E.PLANNING_FAILED: "no plan found (try another target or planner)",
    E.INVALID_MOTION_PLAN: "planned path is in collision (obstacle in the way?)",
    E.CONTROL_FAILED: "controller failed to execute the trajectory",
    E.TIMED_OUT: "timed out",
    E.PREEMPTED: "stopped (cancel or new goal)",
    E.START_STATE_IN_COLLISION: "robot is currently in collision",
    E.GOAL_IN_COLLISION: "target is in collision",
    E.GOAL_STATE_INVALID: "target unreachable or in collision (check pose / obstacles)",
    E.NO_IK_SOLUTION: "target out of reach (no IK solution)",
}
# MoveIt Humble /move_action feedback only sends PLANNING / IDLE; EXECUTING comes from the arm
# controller's trajectory action status instead (T-7).


def stop_deceleration(v_max, deceleration, max_coast, deceleration_max):
    """Harder stop at high speed (V-7): keep the coast v^2 / (2 a) <= max_coast, within [decel, decel_max]."""
    return min(max(deceleration, v_max * v_max / (2.0 * max_coast)), deceleration_max)


def stop_trajectory(joint_names, positions, velocities, deceleration, min_time):
    """2-point deceleration trajectory (decision V-1): (q, v) at t = 0 -> (q + v T / 2, 0) at T.

    T = max_j |v_j| / deceleration (>= min_time), common to all joints. The controller's cubic spline
    through these points gives a linear velocity ramp = constant deceleration (no jump in velocity).
    Returns (FollowJointTrajectory.Goal, T).
    """
    v = list(velocities) if len(velocities) == len(positions) else [0.0] * len(positions)
    duration = max(max(abs(x) for x in v) / deceleration, min_time)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    start = JointTrajectoryPoint(positions=list(positions), velocities=v, time_from_start=Duration())
    end = JointTrajectoryPoint(positions=[q + x * duration / 2 for q, x in zip(positions, v)],
                               velocities=[0.0] * len(v), accelerations=[0.0] * len(v),
                               time_from_start=Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9)))
    goal.trajectory.points = [start, end]
    return goal, duration


def describe(code, detail=""):
    """'<CODE_NAME>: <detail or hint>' (T-6)."""
    return f"{CODE_NAMES.get(code, str(code))}: {detail or HINTS.get(code, 'see MoveItErrorCodes')}"


def rpy_to_quat(roll, pitch, yaw):
    """(x, y, z, w) for R = Rz(yaw) * Ry(pitch) * Rx(roll); inverse of quat_to_rpy."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def down_quat(yaw):
    """(x, y, z, w) for MODE_DOWN_YAW: R = Rz(yaw) * Rx(pi) (F7)."""
    return math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def matrix_to_quat(m):
    """(x, y, z, w) of a rotation matrix (Shepperd's method, numerically stable)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        return (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2 * math.sqrt(1 + m[0][0] - m[1][1] - m[2][2])
        return s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s
    if m[1][1] > m[2][2]:
        s = 2 * math.sqrt(1 + m[1][1] - m[0][0] - m[2][2])
        return (m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s
    s = 2 * math.sqrt(1 + m[2][2] - m[0][0] - m[1][1])
    return (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def delta_target(position, quat, d, drpy, frame):
    """Target (position, quat) of a relative move (frozen semantics, decision U-2).

    D = translation d then rotation Rd = Rz(dyaw) Ry(dpitch) Rx(droll).
    FRAME_TCP:  p' = p + R d, R' = R Rd (along / about the tcp's own axes).
    FRAME_BASE: p' = p + d,   R' = Rd R (about base axes, pivoting at the tcp point).
    """
    r = quat_to_matrix(*quat)
    rd = quat_to_matrix(*rpy_to_quat(*drpy))
    if frame == MoveDelta.Goal.FRAME_TCP:
        step = [sum(r[i][k] * d[k] for k in range(3)) for i in range(3)]
        rot = _matmul(r, rd)
    else:
        step = list(d)
        rot = _matmul(rd, r)
    return [position[i] + step[i] for i in range(3)], matrix_to_quat(rot)

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
    "joint_tolerance": Parameter.Type.DOUBLE,
    "server_timeout": Parameter.Type.DOUBLE,
    "state_timeout": Parameter.Type.DOUBLE,
    "trajectory_status_topic": Parameter.Type.STRING,
    "controller_action": Parameter.Type.STRING,
    "controller_state_topic": Parameter.Type.STRING,
    "stop_deceleration": Parameter.Type.DOUBLE,
    "stop_max_coast": Parameter.Type.DOUBLE,
    "stop_deceleration_max": Parameter.Type.DOUBLE,
    "stop_min_time": Parameter.Type.DOUBLE,
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


def quat_to_rpy(x, y, z, w):
    """(roll, pitch, yaw) for R = Rz(yaw) * Ry(pitch) * Rx(roll) (fixed axes X, Y, Z).

    Tool pointing down gives roll = +-pi, pitch = 0. At gimbal lock (pitch = +-pi/2) yaw is
    set to 0 and roll takes the whole rotation about the vertical. NOTE: convention to be
    checked against the real robot / pendant (decision-log T12).
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + w * z)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    if math.cos(pitch) > 1e-6:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:  # gimbal lock
        r11 = 1 - 2 * (x * x + z * z)
        r12 = 2 * (y * z - w * x)
        roll, yaw = math.atan2(-r12, r11), 0.0
    return roll, pitch, yaw


def chain_joints(urdf_xml, base, tip):
    """Movable joint names from base to tip (URDF order), e.g. ['j1', ..., 'j6']."""
    # top-level <joint> only: <ros2_control> also contains <joint> elements (without <child>)
    parent_joint = {j.find("child").get("link"): j for j in ET.fromstring(urdf_xml).findall("joint")}
    names, link = [], tip
    while link != base:
        joint = parent_joint[link]  # KeyError -> tip not below base
        if joint.get("type") != "fixed":
            names.append(joint.get("name"))
        link = joint.find("parent").get("link")
    return list(reversed(names))


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
        self.joint_names = None    # planning chain joints, filled from the URDF (S-4)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description_semantic", self._on_srdf, latched)
        self.create_subscription(String, "/robot_description", self._on_urdf, latched)

        self._last_js = None       # latest /joint_states and its arrival time (S-2)
        self._last_js_time = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)  # runs on this executor (S-7)

        self._move_group = ActionClient(self, MoveGroup, self.cfg["move_group_action"])
        self._moveit_goal = None   # handle of the running MoveIt goal (to cancel it)
        self._moveit_owner = None  # our goal handle that owns the running MoveIt goal
        self._on_executing = None  # set while a MoveIt goal runs: publishes EXECUTING (T-7)
        self._controller_executing = False  # arm controller is executing a trajectory
        self._ctrl_state = None    # latest controller_state (commanded reference) + receipt time
        self._stop_sent = False    # smooth stop already sent for the running MoveIt goal
        self._stop_goal = None     # future of the stop trajectory goal (to wait for standstill)
        self._controller = ActionClient(self, FollowJointTrajectory, self.cfg["controller_action"])
        self.create_subscription(JointTrajectoryControllerState, self.cfg["controller_state_topic"],
                                 self._on_controller_state, 10)
        self.create_subscription(GoalStatusArray, self.cfg["trajectory_status_topic"], self._on_trajectory_status, 10)
        self._stopped = set()      # goal ids that were cancelled or preempted

        self._active = None  # goal handle of the running goal (both actions)
        self._move_to = ActionServer(self, MoveTo, "move_to", self._execute_move_to,
                                     goal_callback=self._accept, cancel_callback=self._cancel,
                                     handle_accepted_callback=self._start)
        self._move_delta = ActionServer(self, MoveDelta, "move_delta", self._execute_move_delta,
                                        goal_callback=self._accept, cancel_callback=self._cancel,
                                        handle_accepted_callback=self._start)
        self.create_service(GetState, "get_state", self._get_state)
        self.get_logger().info(f"ready: {self.get_namespace()}/move_to, move_delta, get_state | config {self.cfg}")

    # ---- robot model + state ----------------------------------------------------------------
    def _on_srdf(self, msg):
        self.named_targets = group_states(msg.data, self.cfg["group"])
        self.get_logger().info(f"SRDF received, named targets for '{self.cfg['group']}': "
                               f"{sorted(self.named_targets)}")

    def _on_urdf(self, msg):
        try:
            self.joint_names = chain_joints(msg.data, self.cfg["base_frame"], self.cfg["tcp_link"])
            self.get_logger().info(f"URDF received, joints {self.cfg['base_frame']} -> "
                                   f"{self.cfg['tcp_link']}: {self.joint_names}")
        except (KeyError, ET.ParseError) as e:
            self.get_logger().error(f"cannot find chain {self.cfg['base_frame']} -> {self.cfg['tcp_link']} "
                                    f"in /robot_description: {e!r}")

    def _on_joint_states(self, msg):
        self._last_js = msg
        self._last_js_time = self.get_clock().now()

    def _on_trajectory_status(self, msg):
        """Arm controller executing a trajectory -> EXECUTING phase for our running goal (T-7)."""
        self._controller_executing = any(s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list)
        if not self._controller_executing:
            return
        if self._on_executing:
            self._on_executing()
        # stopped while planning: MoveIt executes the plan anyway -> stop it at once (V-3)
        if self._moveit_owner is not None and self._is_stopped(self._moveit_owner):
            self._smooth_stop()

    def _on_controller_state(self, msg):
        self._ctrl_state = (msg, time.monotonic())

    def _smooth_stop(self):
        """Send the deceleration trajectory once per MoveIt goal (V-1); MoveIt then aborts its execution."""
        if self._stop_sent:
            return
        if self._ctrl_state is None:
            self.get_logger().error("smooth stop impossible: no controller_state received")
            return
        msg, received = self._ctrl_state
        age = time.monotonic() - received
        v = list(msg.reference.velocities)
        if len(v) != len(msg.reference.positions):
            v = [0.0] * len(msg.reference.positions)
        q = [p + x * age for p, x in zip(msg.reference.positions, v)]  # extrapolate to now (V-5)
        v_max = max((abs(x) for x in v), default=0.0)
        decel = stop_deceleration(v_max, self.cfg["stop_deceleration"], self.cfg["stop_max_coast"],
                                  self.cfg["stop_deceleration_max"])
        goal, duration = stop_trajectory(msg.joint_names, q, v, decel, self.cfg["stop_min_time"])
        self._stop_sent = True
        self._stop_goal = self._controller.send_goal_async(goal)
        self.get_logger().info(f"smooth stop: |v|max {v_max:.3f} rad/s, decel {decel:.2f} rad/s^2, "
                               f"{duration * 1000:.0f} ms, coast {v_max * duration / 2:.3f} rad, state age {age * 1000:.0f} ms")

    def read_state(self):
        """(code, message, JointState, PoseStamped, (roll, pitch, yaw)); code SUCCESS if valid (S-3)."""
        joints, pose, rpy = JointState(), PoseStamped(), (0.0, 0.0, 0.0)
        if self.joint_names is None:
            return E.COMMUNICATION_FAILURE, "no /robot_description yet", joints, pose, rpy
        if self._last_js is None:
            return E.ROBOT_STATE_STALE, "no /joint_states received yet", joints, pose, rpy
        age = (self.get_clock().now() - self._last_js_time).nanoseconds * 1e-9
        if age > self.cfg["state_timeout"]:
            return (E.ROBOT_STATE_STALE, f"latest /joint_states is {age:.1f} s old "
                    f"(> state_timeout {self.cfg['state_timeout']} s)", joints, pose, rpy)
        positions = dict(zip(self._last_js.name, self._last_js.position))
        missing = [j for j in self.joint_names if j not in positions]
        if missing:
            return E.ROBOT_STATE_STALE, f"/joint_states lacks joints {missing}", joints, pose, rpy
        base, tcp = self.cfg["base_frame"], self.cfg["tcp_link"]
        try:
            tf = self._tf_buffer.lookup_transform(base, tcp, Time())  # latest (S-1)
        except TransformException as e:
            return E.FRAME_TRANSFORM_FAILURE, f"TF {base} -> {tcp}: {e}", joints, pose, rpy
        joints.header = self._last_js.header
        joints.name = list(self.joint_names)  # chain order j1..j6 (S-4)
        joints.position = [positions[j] for j in self.joint_names]
        pose.header = tf.header  # transform's own stamp (S-6)
        t, q = tf.transform.translation, tf.transform.rotation
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = q
        return E.SUCCESS, f"state age {age * 1000:.0f} ms", joints, pose, quat_to_rpy(q.x, q.y, q.z, q.w)

    # ---- goal handling (shared by both actions) ----------------------------------------------
    def _accept(self, _goal_request):
        return GoalResponse.ACCEPT  # invalid goals are aborted with a reason in execute (C-3)

    def _cancel(self, goal_handle):
        self.get_logger().info("cancel requested")
        self._stop(goal_handle)
        return CancelResponse.ACCEPT

    def _start(self, goal_handle):
        if self._active is not None and self._active.is_active:
            self.get_logger().info("new goal received: preempting the running goal")
            self._stop(self._active)
        self._active = goal_handle
        goal_handle.execute()

    def _stop(self, goal_handle):
        """Mark a goal as stopped and cancel its MoveIt goal (cancel / preempt, C-2)."""
        self._stopped.add(bytes(goal_handle.goal_id.uuid))
        if self._moveit_owner is goal_handle and self._moveit_goal is not None:
            self._moveit_goal.cancel_goal_async()  # MoveIt Humble does not stop execution on this (5j)
            if self._controller_executing:
                self._smooth_stop()  # otherwise done when execution starts (V-3)

    def _is_stopped(self, goal_handle):
        return bytes(goal_handle.goal_id.uuid) in self._stopped

    def _finish(self, goal_handle, result_type, code, detail=""):
        if self._is_stopped(goal_handle) and code != E.SUCCESS:
            code, detail = E.PREEMPTED, ""
        message = describe(code, detail)
        result = result_type(success=(code == E.SUCCESS), message=message, error_code=E(val=code))
        state_code, _, _, pose, _ = self.read_state()
        if state_code == E.SUCCESS:
            result.final_tcp_pose = pose
        else:
            result.final_tcp_pose.header.frame_id = self.cfg["base_frame"]
        if code == E.SUCCESS:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._stopped.discard(bytes(goal_handle.goal_id.uuid))
        if self._active is goal_handle:
            self._active = None
        # rclpy fixes the severity per call site: one call per severity
        text = f"{result_type.__module__.split('.')[-1]}: code {code}: {message}"
        if code == E.SUCCESS:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        return result

    # ---- MoveIt ------------------------------------------------------------------------------
    async def _sleep(self, seconds):
        """Non-blocking sleep inside an async callback (timer-backed future)."""
        done = Future()
        timer = self.create_timer(seconds, lambda: done.done() or done.set_result(None))
        await done
        self.destroy_timer(timer)

    async def _wait_moveit_idle(self):
        """A preempted goal's MoveIt goal must finish before the next one is sent."""
        waited = 0.0
        while self._moveit_goal is not None and waited < self.cfg["server_timeout"]:
            await self._sleep(0.02)
            waited += 0.02
        return self._moveit_goal is None

    async def _wait_server(self):
        waited = 0.0
        while not self._move_group.server_is_ready() and waited < self.cfg["server_timeout"]:
            await self._sleep(0.1)
            waited += 0.1
        return self._move_group.server_is_ready()

    async def _prepare(self):
        """Common checks before planning (U-5): (code, message) on failure, else None."""
        if not await self._wait_moveit_idle():
            return E.TIMED_OUT, "previous MoveIt goal did not stop"
        code, message, _, _, _ = self.read_state()
        if code != E.SUCCESS:
            return code, message
        if not await self._wait_server():
            return E.COMMUNICATION_FAILURE, f"{self.cfg['move_group_action']} not available (is move_group running?)"
        return None

    def _request(self, goal, constraints, planner, scaling):
        """MoveGroup goal (request + options) for the arm group."""
        mg = MoveGroup.Goal()
        r = mg.request
        r.group_name = self.cfg["group"]
        r.pipeline_id, r.planner_id = PIPELINES[planner]
        r.num_planning_attempts = self.cfg["planning_attempts"]
        r.allowed_planning_time = self.cfg["planning_time"]
        scaling = scaling if scaling > 0.0 else self.cfg["default_velocity_scaling"]  # F14
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = scaling
        r.start_state.is_diff = True  # plan from the current state
        r.goal_constraints = [constraints]
        mg.planning_options.plan_only = goal.plan_only
        mg.planning_options.planning_scene_diff.is_diff = True
        mg.planning_options.planning_scene_diff.robot_state.is_diff = True
        return mg

    def _pose_constraints(self, x, y, z, quat):
        """Goal sphere (position_tolerance) + orientation (orientation_tolerance) for the tcp."""
        base, tcp = self.cfg["base_frame"], self.cfg["tcp_link"]
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = x, y, z
        centre.orientation.w = 1.0
        pc = PositionConstraint(link_name=tcp, weight=1.0)
        pc.header.frame_id = base
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.cfg["position_tolerance"]])],
            primitive_poses=[centre])
        tol = self.cfg["orientation_tolerance"]
        oc = OrientationConstraint(link_name=tcp, weight=1.0, absolute_x_axis_tolerance=tol,
                                   absolute_y_axis_tolerance=tol, absolute_z_axis_tolerance=tol)
        oc.header.frame_id = base
        n = math.sqrt(sum(v * v for v in quat))
        # all 4 components set explicitly: Quaternion defaults to w = 1 (Q-1)
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = (v / n for v in quat)
        return Constraints(position_constraints=[pc], orientation_constraints=[oc])

    def _joint_constraints(self, targets):
        tol = self.cfg["joint_tolerance"]
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=tol, tolerance_below=tol, weight=1.0)
            for j, v in targets.items()])

    async def _run_moveit(self, goal_handle, moveit_goal, feedback_type, retry):
        """Send to /move_action (with OMPL retries); returns the MoveIt error code."""
        last_phase = ["PLANNING"]  # already published by the caller

        def publish(phase):
            if phase != last_phase[0] and goal_handle.is_active:
                last_phase[0] = phase
                goal_handle.publish_feedback(feedback_type(phase=phase))

        def on_feedback(msg):  # MoveIt: PLANNING at start (again on retries), IDLE at the end
            if msg.feedback.state == "PLANNING":
                publish("PLANNING")

        self._on_executing = lambda: publish("EXECUTING")

        tries = 1 + (self.cfg["replan_retries"] if retry else 0)
        code = E.FAILURE
        for attempt in range(1, tries + 1):
            if self._is_stopped(goal_handle):
                return E.PREEMPTED
            self._stop_sent, self._stop_goal = False, None
            send = self._move_group.send_goal_async(moveit_goal, feedback_callback=on_feedback)
            handle = await send
            if not handle.accepted:
                return E.COMMUNICATION_FAILURE
            self._moveit_goal, self._moveit_owner = handle, goal_handle
            if self._is_stopped(goal_handle):  # stopped while the goal was being sent
                handle.cancel_goal_async()
            response = await handle.get_result_async()
            if self._stop_goal is not None:  # wait until the stop trajectory has finished (V-2)
                stop_handle = await self._stop_goal
                if stop_handle.accepted:
                    await stop_handle.get_result_async()
            self._moveit_goal, self._moveit_owner = None, None
            self._on_executing = None
            code = response.result.error_code.val
            if code in RETRY_CODES and attempt < tries and not self._is_stopped(goal_handle):
                self.get_logger().warn(f"attempt {attempt}/{tries}: {describe(code)} -> planning again")
                continue
            return code
        return code

    # ---- actions -----------------------------------------------------------------------------
    async def _execute_move_to(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"move_to: named_target='{g.named_target}' xyz=({g.x:.4f}, {g.y:.4f}, {g.z:.4f}) "
            f"rpy=({g.roll:.3f}, {g.pitch:.3f}, {g.yaw:.3f}) mode={g.mode} planner='{g.planner}' "
            f"scaling={g.velocity_scaling} plan_only={g.plan_only}")
        error = validate_move_to(g, self.named_targets) or await self._prepare()
        if error:
            return self._finish(goal_handle, MoveTo.Result, *error)

        if g.named_target:
            constraints = self._joint_constraints(self.named_targets[g.named_target])
        else:
            quat = down_quat(g.yaw) if g.mode == MoveTo.Goal.MODE_DOWN_YAW else rpy_to_quat(g.roll, g.pitch, g.yaw)
            constraints = self._pose_constraints(g.x, g.y, g.z, quat)
        goal_handle.publish_feedback(MoveTo.Feedback(phase="PLANNING"))
        code = await self._run_moveit(goal_handle, self._request(g, constraints, g.planner, g.velocity_scaling),
                                      MoveTo.Feedback, retry=(g.planner == ""))
        detail = ("plan found (plan_only, not executed)" if g.plan_only else "target reached") \
            if code == E.SUCCESS else ""
        return self._finish(goal_handle, MoveTo.Result, code, detail)

    async def _execute_move_delta(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"move_delta: d=({g.dx:.4f}, {g.dy:.4f}, {g.dz:.4f}) drpy=({g.droll:.3f}, {g.dpitch:.3f}, "
            f"{g.dyaw:.3f}) frame={g.frame} scaling={g.velocity_scaling} plan_only={g.plan_only}")
        error = validate_move_delta(g)
        if error:
            return self._finish(goal_handle, MoveDelta.Result, *error)
        d, drpy = (g.dx, g.dy, g.dz), (g.droll, g.dpitch, g.dyaw)
        if all(v == 0.0 for v in d + drpy):  # nothing to do (C-9, U-3)
            return self._finish(goal_handle, MoveDelta.Result, E.SUCCESS, "zero delta, no motion")
        error = await self._prepare()
        if error:
            return self._finish(goal_handle, MoveDelta.Result, *error)
        _, _, _, pose, _ = self.read_state()  # current tcp pose (U-1)
        p, q = pose.pose.position, pose.pose.orientation
        target, quat = delta_target((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), d, drpy, g.frame)
        self.get_logger().info(f"move_delta target: xyz=({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}) "
                               f"rpy={tuple(round(v, 3) for v in quat_to_rpy(*quat))}")
        goal_handle.publish_feedback(MoveDelta.Feedback(phase="PLANNING"))
        constraints = self._pose_constraints(*target, quat)
        code = await self._run_moveit(goal_handle, self._request(g, constraints, "LIN", g.velocity_scaling),
                                      MoveDelta.Feedback, retry=False)  # always Pilz LIN, deterministic (U-4)
        detail = ("plan found (plan_only, not executed)" if g.plan_only else "target reached") \
            if code == E.SUCCESS else ""
        return self._finish(goal_handle, MoveDelta.Result, code, detail)

    # ---- service -----------------------------------------------------------------------------
    def _get_state(self, _request, response):
        code, message, joints, pose, (roll, pitch, yaw) = self.read_state()
        response.success = code == E.SUCCESS
        response.message = message
        response.joints, response.tcp_pose = joints, pose
        response.roll, response.pitch, response.yaw = roll, pitch, yaw
        if not response.success:
            self.get_logger().warn(f"get_state: {message}")
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C: rclpy's signal handler may shut down first -> ExternalShutdownException
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
