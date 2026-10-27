"""skill_server node (T4, control-loop.md section 4; decision-log TM-10): robot skills as ROS 2 actions.

    /task/skills/pick          Pick(board_id, contact_search, velocity_scaling)
    /task/skills/place         Place(target zone | x, y, yaw)
    /task/skills/home          Home
    /task/skills/wait_clear    WaitClear(timeout)
    /task/skills/release_safe  ReleaseSafe(target "" = fault zone)

Uses only our APIs: /motion/move_to + move_delta, /gripper/grip + release + state, /scene/attach_board + detach_board,
/task/world (facts), and publishes what it did on /task/events (picked / placed / dropped). One skill at a time.
Every motion is watched: obstacle in the work area -> smooth stop, OBSTACLE; vacuum lost while holding -> smooth stop,
LOST_IN_TRANSIT; cancel -> smooth stop, CANCELLED.
"""
import json
import math
import os
import signal
import threading
import time

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_interfaces.action import Grip, MoveDelta, MoveTo, Release
from hk_fr5_interfaces.msg import GripperState
from hk_fr5_interfaces.srv import AddBox, AttachBoard, DetachBoard, RemoveBox
from hk_fr5_task import skill_math as sm
from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear
from hk_fr5_task_interfaces.msg import SkillCode, World

VERBOSE = os.environ.get("HK_VERBOSE", "0") == "1"       # TM-27: tools/live_env.sh debug -> every step logged
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
C = SkillCode
EXEC_ERRORS = (-4, -6, -7)          # MoveItErrorCodes CONTROL_FAILED, TIMED_OUT, PREEMPTED -> MOTION_FAILED


class SkillError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server", namespace="task")
        self.declare_parameter("config", os.path.join(get_package_share_directory("hk_fr5_task"), "config", "skills.yaml"))
        with open(self.get_parameter("config").value) as f:
            self.c = yaml.safe_load(f)
        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.move_to = ActionClient(self, MoveTo, "/motion/move_to", callback_group=cb)
        self.move_delta = ActionClient(self, MoveDelta, "/motion/move_delta", callback_group=cb)
        self.grip = ActionClient(self, Grip, "/gripper/grip", callback_group=cb)
        self.release = ActionClient(self, Release, "/gripper/release", callback_group=cb)
        self.attach = self.create_client(AttachBoard, "/scene/attach_board", callback_group=cb)
        self.detach = self.create_client(DetachBoard, "/scene/detach_board", callback_group=cb)
        self.add_box = self.create_client(AddBox, "/scene/add_box", callback_group=cb)
        self.remove = self.create_client(RemoveBox, "/scene/remove_box", callback_group=cb)
        self.boxes = None                                # W13: zones with a stack box now (None = unknown at start)
        self.world, self.world_t, self.gripper = None, 0.0, None
        self.create_subscription(World, "world", self._on_world, LATCHED, callback_group=cb)
        self.create_subscription(GripperState, "/gripper/state", lambda m: setattr(self, "gripper", m), LATCHED,
                                 callback_group=cb)
        self.events = self.create_publisher(String, "events", 50)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.busy = ""
        for name, typ, fn in (("pick", Pick, self._pick), ("place", Place, self._place), ("home", Home, self._home),
                              ("wait_clear", WaitClear, self._wait_clear), ("release_safe", ReleaseSafe, self._release_safe)):
            ActionServer(self, typ, f"skills/{name}", execute_callback=self._runner(name, typ, fn),
                         goal_callback=self._goal_cb(name), cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=cb)
        self.get_logger().info("skills ready: /task/skills/{pick, place, home, wait_clear, release_safe}")

    # ---------- plumbing ----------
    def _step(self, text):
        """TM-26: one INFO line per step of a skill (motion goal, gripper, scene, world event, phase) -> /rosout."""
        if VERBOSE:
            self.get_logger().info(f"[{self.busy or '-'}] {text}")

    def _step_warn(self, text):
        """Same, WARN (its own call site: rclpy logger severity trap)."""
        self.get_logger().warn(f"[{self.busy or '-'}] {text}")

    def _on_world(self, m):
        self.world, self.world_t = m, time.monotonic()

    def _goal_cb(self, name):
        def cb(_goal):
            with self.lock:
                if self.busy:
                    self.get_logger().warn(f"{name} refused: '{self.busy}' is running (one skill at a time)")
                    return GoalResponse.REJECT
                self.busy = name
            return GoalResponse.ACCEPT
        return cb

    def _runner(self, name, typ, fn):
        def execute(gh):
            self.fb_type = typ                             # feedback type of the running action (one at a time)
            res = typ.Result()
            t_start = time.monotonic()
            req = gh.request
            self._step(f"START goal {{{', '.join(f'{k}={getattr(req, k)!r}' for k in req.get_fields_and_field_types())}}}")
            try:
                msg = fn(gh, res) or "done"
                res.success, res.code, res.message = True, SkillCode(value=C.SUCCESS), msg
                gh.succeed()
            except SkillError as e:
                res.success, res.code, res.message = False, SkillCode(value=e.code), str(e)
                gh.canceled() if e.code == C.CANCELLED else gh.abort()
            except Exception as e:                       # never leave a goal hanging
                self.get_logger().error(f"{name}: internal error: {e!r}")
                res.success, res.code, res.message = False, SkillCode(value=C.INTERNAL_ERROR), f"internal error: {e!r}"
                gh.abort()
            finally:
                with self.lock:
                    self.busy = ""
            if res.success:                                # one call site per severity (rclpy raises otherwise)
                self.get_logger().info(f"{name}: OK - {res.message} ({time.monotonic() - t_start:.1f} s)")
            else:
                self.get_logger().warn(f"{name}: FAILED - {res.message} ({time.monotonic() - t_start:.1f} s)")
            return res
        return execute

    def _phase(self, gh, _typ, phase):
        self._step(f"phase {phase}")
        fb = self.fb_type.Feedback()
        if hasattr(fb, "phase"):
            fb.phase = phase
            gh.publish_feedback(fb)

    def _event(self, **ev):
        self._step(f"world event {ev}")
        self.events.publish(String(data=json.dumps(ev)))

    def _wait_future(self, fut, timeout, gh=None):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                return False
            if gh is not None and gh.is_cancel_requested:
                return False
            time.sleep(0.01)
        return True

    def _fresh_world(self):
        w = self.world
        if w is None or time.monotonic() - self.world_t > self.c["fresh"] or not w.vision_ok:
            raise SkillError(C.VISION_STALE, "no fresh world / Vision 1 data")
        return w

    def _holding(self):
        g = self.gripper
        return g is not None and g.state == GripperState.HOLDING

    def _obstacle(self):
        w = self.world
        return w is not None and any(o.in_work_area and o.age < self.c["fresh"] for o in w.obstacles)

    def _tcp(self):
        try:
            t = self.tf.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform.translation
            return t.x, t.y, t.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            raise SkillError(C.INTERNAL_ERROR, f"no tcp pose from TF: {e}")

    def _grip_log(self, how, attempt, gres, gripped):
        r = gres.result().result if gres.done() else None
        self._step(f"grip ({how}, attempt {attempt + 1}): {'SEALED' if gripped else 'NO SEAL'} at {self._tcp_text()}"
                   + (f" - {r.message} (vacuum after {getattr(r, 'time_to_vacuum', float('nan')):.2f} s)" if r else
                      " - no grip result"))

    def _tcp_text(self):
        try:
            return "tcp (%.4f, %.4f, %.4f)" % self._tcp()
        except SkillError:
            return "tcp unknown"

    def _call(self, client, req, what, timeout=5.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise SkillError(C.SCENE_ERROR, f"{what}: service not available")
        fut = client.call_async(req)
        if not self._wait_future(fut, timeout):
            raise SkillError(C.SCENE_ERROR, f"{what}: no answer")
        r = fut.result()
        self._step(f"scene {what} {{{', '.join(f'{k}={getattr(req, k)!r}' for k in req.get_fields_and_field_types())}}} "
                   f"-> success={getattr(r, 'success', '?')} {getattr(r, 'message', '')}")
        return r

    # ---------- stack boxes in the scene (W13 / CL-38) ----------
    def _update_stacks(self, skip=""):
        """One box 'stack_<zone>' per drop-off / fault stack, sized from the world (sm.stack_box); `skip` = the board
        about to be picked (not part of its stack any more, motion rule S-11). add_box replaces a box of the same name;
        a zone without a stack loses its box. Rebuilt from the world each time: right after a restart too."""
        w = self.world
        if w is None:
            return
        boards = [{k: getattr(b, k) for k in ("id", "location", "zone", "length", "width", "thickness")} for b in w.boards]
        now = set()
        for z in w.zones:
            zd = {k: getattr(z, k) for k in ("name", "kind", "x", "y", "yaw", "stack_top_z", "base_z")}
            box = sm.stack_box(self.c, zd, boards, skip)
            name = f"stack_{z.name}"
            if box is not None:
                r = self._call(self.add_box, AddBox.Request(name=name, size=box["size"], position=box["position"],
                                                            yaw=float(box["yaw"])), "add_box")
                if not r.success:
                    raise SkillError(C.SCENE_ERROR, f"add_box {name}: {r.message}")
                now.add(z.name)
            elif z.kind != "processing" and (self.boxes is None or z.name in self.boxes):
                self._call(self.remove, RemoveBox.Request(name=name), "remove_box")     # absent at start: fine
        self.boxes = now

    # ---------- watched motion ----------
    def _motion(self, client, goal, gh, holding=False, stop_when=None, what="move", timeout=None):
        """Run one motion goal, watch it. Returns 'done' or 'stopped' (stop_when became true). Raises SkillError.
        timeout: expected duration based (a hung move_group is reported fast, TM-10), capped by motion_timeout."""
        if not client.wait_for_server(timeout_sec=2.0):
            raise SkillError(C.MOTION_FAILED, f"{what}: /motion not available")
        t0 = time.monotonic()
        try:
            here = "tcp (%.4f, %.4f, %.4f)" % self._tcp()
        except SkillError:
            here = "tcp unknown"
        if isinstance(goal, MoveTo.Goal):
            tgt = (f"move_to {goal.named_target}" if goal.named_target else
                   f"move_to ({goal.x:.4f}, {goal.y:.4f}, {goal.z:.4f}) yaw {goal.yaw:.3f} planner '{goal.planner or 'OMPL'}'")
        else:
            tgt = f"move_delta dz {goal.dz * 1000:+.1f} mm (LIN)"
        self._step(f"{what}: {tgt} speed {goal.velocity_scaling:.2f} from {here}{' HOLDING' if holding else ''}"
                   f"{' (stops on a condition)' if stop_when is not None else ''}")
        fut = client.send_goal_async(goal)
        if not self._wait_future(fut, 5.0) or not fut.result().accepted:
            raise SkillError(C.MOTION_FAILED, f"{what}: goal not accepted")
        mg = fut.result()
        rf = mg.get_result_async()
        end, reason = time.monotonic() + min(timeout or self.c["motion_timeout"], self.c["motion_timeout"]), None
        while not rf.done():
            if gh.is_cancel_requested:
                reason = (C.CANCELLED, f"{what}: cancelled")
            elif self._obstacle():
                reason = (C.OBSTACLE, f"{what}: obstacle in the work area")
            elif holding and not self._holding():
                reason = (C.LOST_IN_TRANSIT, f"{what}: vacuum lost while holding")
            elif stop_when is not None and stop_when():
                reason = ("stopped", "")
            elif time.monotonic() > end:
                reason = (C.MOTION_FAILED, f"{what}: no result in time (motion stack hung?)")
            if reason:
                mg.cancel_goal_async()                      # motion controller: smooth stop
                self._wait_future(rf, 5.0)
                if reason[0] == "stopped":
                    self._step(f"{what}: stopped by its condition after {time.monotonic() - t0:.2f} s at {self._tcp_text()}")
                    return "stopped"
                self._step_warn(f"{what}: aborted ({reason[1]}) after {time.monotonic() - t0:.2f} s at {self._tcp_text()}")
                raise SkillError(*reason)
            time.sleep(0.02)
        r = rf.result().result
        if r.success:
            self._step(f"{what}: done in {time.monotonic() - t0:.2f} s, now at {self._tcp_text()}")
            return "done"
        self._step_warn(f"{what}: motion failed after {time.monotonic() - t0:.2f} s: error {r.error_code.val} {r.message}")
        code = C.MOTION_FAILED if r.error_code.val in EXEC_ERRORS else C.PLAN_FAILED
        raise SkillError(code, f"{what}: {r.message}")

    def _goto(self, gh, x, y, z, yaw, holding=False, what="move", retries=None):
        g = MoveTo.Goal(x=float(x), y=float(y), z=float(z), yaw=float(yaw), mode=MoveTo.Goal.MODE_DOWN_YAW,
                        velocity_scaling=float(self.c["speed"]))
        for k in range((self.c["retries"] if retries is None else retries) + 1):
            try:
                return self._motion(self.move_to, g, gh, holding=holding, what=what, timeout=self.c["move_timeout"])
            except SkillError as e:
                if e.code != C.PLAN_FAILED or k == (self.c["retries"] if retries is None else retries):
                    raise

    def _vertical(self, gh, z, speed, holding=False, stop_when=None, what="LIN"):
        """Straight up / down (Pilz LIN) to tcp z (base frame)."""
        for k in range(2):
            dz = z - self._tcp()[2]
            if abs(dz) < 0.0005:
                return "done"
            g = MoveDelta.Goal(dz=float(dz), frame=MoveDelta.Goal.FRAME_BASE, velocity_scaling=float(speed))
            expected = abs(dz) / max(speed * self.c["lin_max_speed"], 1e-3)
            try:
                return self._motion(self.move_delta, g, gh, holding=holding, stop_when=stop_when, what=what,
                                    timeout=3.0 * expected + 8.0)
            except SkillError as e:                         # Pilz LIN refuses when a joint would exceed its
                if e.code != C.PLAN_FAILED or k == 1:       # acceleration limit: once more at half the speed
                    raise
                speed = max(0.01, speed / 2)

    # ---------- skills ----------
    def _pick(self, gh, res):
        req = gh.request
        if self._holding() or (self.world is not None and self.world.held_board_id):
            raise SkillError(C.ALREADY_HOLDING, "a board is already held")
        w = self._fresh_world()
        b = next((b for b in w.boards if b.id == req.board_id), None)
        if b is None or b.location == "held" or b.age > self.c["fresh"]:
            raise SkillError(C.BOARD_NOT_FOUND, f"board '{req.board_id}' not seen recently")
        board = {"x": b.x, "y": b.y, "top_z": b.top_z}
        thick = b.thickness if not math.isnan(b.thickness) else self.c["default_thickness"]
        yaw = sm.tcp_yaw_for(b.yaw, self.c["ref_yaw"])
        hover, end, search = sm.pick_heights(self.c, board)
        search = search or req.contact_search
        speed = req.velocity_scaling or self.c["approach_speed"]
        self._step(f"pick plan: board {b.id} {b.variant} ({b.label}) {b.location}{'/' + b.zone if b.zone else ''} at "
                   f"({b.x:.4f}, {b.y:.4f}) yaw {b.yaw:.3f} size {b.length:.3f} x {b.width:.3f} top_z {b.top_z:.4f} "
                   f"thick {thick:.4f} age {b.age:.2f} s -> tcp yaw {yaw:.3f}, hover {hover:.4f}, "
                   f"{'contact search from ' + format(end, '.4f') + ' to ' + format(sm.search_end_z(self.c), '.4f') if search else 'known top: down to ' + format(end, '.4f')}")
        self._update_stacks(skip=b.id)                   # S-11: the picked board is not part of its stack box
        self._phase(gh, Pick, "HOVER")
        self._goto(gh, b.x, b.y, hover, yaw, what="hover over the board")
        gripped = False
        for attempt in range(self.c["retries"] + 1):
            self._phase(gh, Pick, "DESCEND" if attempt == 0 else "RETRY")
            if search:
                self._vertical(gh, max(end, sm.search_end_z(self.c) + 0.001), speed, what="down to the search start")
                gfut = self.grip.send_goal_async(Grip.Goal(timeout=float(self.c["search_timeout"])))
                if not self._wait_future(gfut, 3.0) or not gfut.result().accepted:
                    raise SkillError(C.GRIPPER_ERROR, "grip goal not accepted")
                gres = gfut.result().get_result_async()
                self._phase(gh, Pick, "GRIP")
                # same depth on a retry: the table is a hard floor (deeper = cup collision box in the table, refused)
                self._vertical(gh, sm.search_end_z(self.c), self.c["search_speed"],
                               stop_when=lambda: gres.done(), what="contact search")
                if not gres.done():                        # reached the bottom without a seal: give it a moment
                    self._wait_future(gres, 0.7, gh)
                if not gres.done():
                    gfut.result().cancel_goal_async()
                    self._wait_future(gres, 2.0)
                if gh.is_cancel_requested:
                    gfut.result().cancel_goal_async()
                    raise SkillError(C.CANCELLED, "cancelled during the contact search")
                gripped = gres.done() and gres.result().result.success
                self._grip_log("contact search" if search else "known top", attempt, gres, gripped)
            else:
                self._vertical(gh, max(end - 0.003 * attempt, sm.search_end_z(self.c)), speed, what="down to the board")
                self._phase(gh, Pick, "GRIP")
                gfut = self.grip.send_goal_async(Grip.Goal(timeout=float(self.c["grip_timeout"])))
                if not self._wait_future(gfut, 3.0) or not gfut.result().accepted:
                    raise SkillError(C.GRIPPER_ERROR, "grip goal not accepted")
                gres = gfut.result().get_result_async()
                self._wait_future(gres, self.c["grip_timeout"] + 2.0, gh)
                gripped = gres.done() and gres.result().result.success
                self._grip_log("contact search" if search else "known top", attempt, gres, gripped)
            if gripped:
                break
            self._vertical(gh, min(hover, self._tcp()[2] + 0.02), speed, what="lift after a failed grip")
        if not gripped:
            self._vertical(gh, hover, speed, what="back to hover")
            raise SkillError(C.GRIP_FAILED, f"no vacuum on board {b.id} after {self.c['retries'] + 1} attempt(s)")
        self._phase(gh, Pick, "LIFT")
        try:
            # first a short straight lift WITHOUT the board in the scene: in contact the cup is pressed into the board,
            # an attached box would reach into the table / stack -> every plan refused (motion branch S-12)
            self._vertical(gh, self._tcp()[2] + self.c["unstick_lift"], speed, holding=True, what="unstick lift")
        except SkillError as e:
            if e.code == C.LOST_IN_TRANSIT:
                self._lost(b.id)
                raise SkillError(C.LOST_ON_LIFT, f"board {b.id} lost while lifting")
            raise
        self._phase(gh, Pick, "ATTACH")
        r = self._call(self.attach, AttachBoard.Request(length=float(b.length), width=float(b.width), height=float(thick),
                                                        yaw=0.0, dx=0.0, dy=0.0), "attach_board")
        attached = r.success
        self._event(type="picked", board=b.id, x=b.x, y=b.y)
        self._phase(gh, Pick, "LIFT")
        try:
            self._vertical(gh, hover, speed, holding=True, what="lift")
        except SkillError as e:
            if e.code == C.LOST_IN_TRANSIT:
                self._lost(b.id)
                raise SkillError(C.LOST_ON_LIFT, f"board {b.id} lost while lifting")
            raise
        if not attached:
            raise SkillError(C.SCENE_ERROR, f"board {b.id} held but not attached in the scene: {r.message}")
        res.board_id, res.variant, res.attach_yaw = b.id, b.variant, 0.0
        return f"picked {b.id} ({b.variant}){' by contact search' if search else ''}"

    def _lost(self, board_id):
        """Vacuum lost: board gone from the cup -> scene detach (drop it), world event, stop the pump."""
        try:
            self._call(self.detach, DetachBoard.Request(keep_as_obstacle=False), "detach_board")
        except SkillError:
            pass
        self._event(type="dropped", board=board_id)
        self.release.send_goal_async(Release.Goal())

    def _place(self, gh, res, target=None):
        req = gh.request
        target = req.target if target is None else target
        w = self.world
        held = w.held_board_id if w is not None else ""
        if not self._holding() or not held:
            raise SkillError(C.NOT_HOLDING, "no board on the cup")
        hb = next((b for b in w.boards if b.id == held), None)
        thick = hb.thickness if hb is not None and not math.isnan(hb.thickness) else self.c["default_thickness"]
        if target:
            z = next((z for z in w.zones if z.name == target), None)
            if z is None or (not z.fixed and z.age > self.c["fresh"]):
                raise SkillError(C.TARGET_UNKNOWN, f"zone '{target}' not known / not seen recently")
            zd = {k: getattr(z, k) for k in ("kind", "x", "y", "yaw", "state", "stack_count", "stack_top_z", "base_z")}
            why = sm.zone_accepts(self.c, zd)
            if why:
                raise SkillError(getattr(C, why), f"zone {target} cannot take a board now ({z.state})")
            x, y, zyaw, tz = sm.place_target(self.c, zd, thick)
            void = zd["kind"] == "fault"                 # CL-39: drop from fault_drop_z, the board leaves the world
            self._step(f"place plan: {held} ({hb.variant if hb else '?'}) thick {thick:.4f} -> zone {target} {zd['kind']} "
                       f"{zd['state']} count {zd['stack_count']} stack_top {zd['stack_top_z']:.4f} base {zd['base_z']:.4f} "
                       f"at ({x:.4f}, {y:.4f}) zone yaw {zyaw:.3f}: release at tcp z {tz:.4f}"
                       f"{' (fault void: fault_drop_z)' if void else ' (press ' + format(self.c['place_press'] * 1000, '.1f') + ' mm)'}")
        else:
            x, y, zyaw = req.x, req.y, req.yaw
            tz, void = self.c["table_z"] + thick - self.c["place_press"], False
            self._step(f"place plan: {held} at the free pose ({x:.4f}, {y:.4f}) yaw {zyaw:.3f}: release at tcp z {tz:.4f}")
        yaw = sm.tcp_yaw_for(zyaw, self.c["ref_yaw"])
        self._update_stacks()
        self._phase(gh, Place, "HOVER")
        try:
            self._goto(gh, x, y, self.c["hover_z"], yaw, holding=True, what=f"carry to {target or 'the pose'}")
            self._phase(gh, Place, "DESCEND")
            self._vertical(gh, tz, self.c["approach_speed"], holding=True, what="down to place")
        except SkillError as e:
            if e.code == C.LOST_IN_TRANSIT:
                self._lost(held)
            raise
        self._phase(gh, Place, "RELEASE")
        rfut = self.release.send_goal_async(Release.Goal(blow_off_time=float(self.c["blow_off_time"])))
        if not self._wait_future(rfut, 3.0) or not rfut.result().accepted:
            raise SkillError(C.RELEASE_FAILED, "release goal not accepted")
        rres = rfut.result().get_result_async()
        ok_r = self._wait_future(rres, self.c["blow_off_time"] + 5.0) and rres.result().result.success
        self._step(f"release (blow-off {self.c['blow_off_time']:.1f} s) at {self._tcp_text()}: "
                   f"{'OK' if ok_r else 'FAILED'} {rres.result().result.message if rres.done() else 'no result'}")
        if not ok_r:
            raise SkillError(C.RELEASE_FAILED, "release failed")
        self._phase(gh, Place, "DETACH")
        r = self._call(self.detach, DetachBoard.Request(keep_as_obstacle=False), "detach_board")   # W13: stack box instead
        self._event(type="placed", board=held, zone=target, x=x, y=y, yaw=sm.wrap_pi(yaw),
                    variant=hb.variant if hb is not None else "")
        self._phase(gh, Place, "RETRACT")
        self._vertical(gh, self.c["hover_z"], self.c["approach_speed"], what="retract")
        if not r.success:
            raise SkillError(C.SCENE_ERROR, f"placed, but detach_board failed: {r.message}")
        if void:                                         # nothing to confirm: the world forgets boards in the void
            self._home_move(gh)
            res.target, res.stack_count = target, 0
            return f"dropped {held} into the fault zone {target} (void)"
        self._phase(gh, Place, "VERIFY")
        self._home_move(gh)                              # out of the camera's way (CL-20 settle check)
        t_end, t_placed = time.monotonic() + self.c["verify_timeout"], time.monotonic()
        seen = False
        while time.monotonic() < t_end and not seen:
            w = self.world
            seen = w is not None and self.world_t > t_placed and any(
                b.id == held and math.hypot(b.x - x, b.y - y) <= self.c["verify_tol"] and b.age < 0.5 for b in w.boards)
            time.sleep(0.05)
        z = next((z for z in self.world.zones if z.name == target), None) if target else None
        res.target, res.stack_count = target, z.stack_count if z is not None else 0
        self._update_stacks()                            # the stack now includes this board
        if not seen:
            raise SkillError(C.NOT_VERIFIED, f"placed {held}, but the camera does not confirm it at the target")
        return f"placed {held} in {target or 'the pose'}"

    def _home_move(self, gh):
        hp = self.c.get("home_pose") or []                # TM-25: park pose (arm out of the overhead view), [] -> 'ready'
        if hp:
            g = MoveTo.Goal(x=float(hp[0]), y=float(hp[1]), z=float(hp[2]), yaw=float(hp[3]),
                            mode=MoveTo.Goal.MODE_DOWN_YAW, velocity_scaling=float(self.c["speed"]))
        else:
            g = MoveTo.Goal(named_target="ready", velocity_scaling=float(self.c["speed"]))
        return self._motion(self.move_to, g, gh, holding=self._holding(), what="home", timeout=self.c["move_timeout"])

    def _home(self, gh, res):
        self._update_stacks()                            # W13: correct stack boxes also right after a restart
        self._phase(gh, Home, "MOVING")
        self._home_move(gh)
        return "at home"

    def _wait_clear(self, gh, res):
        timeout = gh.request.timeout or self.c["clear_timeout"]
        end, clear_since = time.monotonic() + timeout, None
        while time.monotonic() < end:
            if gh.is_cancel_requested:
                raise SkillError(C.CANCELLED, "cancelled")
            n = sum(1 for o in (self.world.obstacles if self.world else []) if o.in_work_area and o.age < self.c["fresh"])
            fb = WaitClear.Feedback(); fb.obstacles = n; gh.publish_feedback(fb)
            if n == 0:
                clear_since = clear_since or time.monotonic()
                if time.monotonic() - clear_since >= self.c["clear_time"]:
                    return "work area clear"
            else:
                clear_since = None
            time.sleep(0.05)
        raise SkillError(C.TIMEOUT, f"work area not clear after {timeout:.0f} s")

    def _release_safe(self, gh, res):
        held = self.world.held_board_id if self.world is not None else ""
        if held and not self._holding():                 # CL-32: taken off the cup by hand (e.g. during a stop)
            raise SkillError(C.NOT_HOLDING, f"the world holds {held} but the cup is empty")
        msg = "nothing held"
        if self._holding() and held:
            target = gh.request.target or "fault"
            msg = self._place(gh, Place.Result(), target=target)
        self._home_move(gh)
        return msg + ", at home"


def main():
    rclpy.init()
    try:
        node = SkillServer()
    except (OSError, KeyError, ValueError) as e:
        print(f"[skill_server] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # TM-27: a 2nd Ctrl+C (terminal + pixi) must not break the shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
