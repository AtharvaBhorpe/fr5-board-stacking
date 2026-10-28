"""scene_manager: owns the MoveIt planning scene objects. Decisions: decision-log 5n (S-*), 5n-1 (model, E-*).

    ros2 run hk_fr5_motion scene_manager --ros-args --params-file <hk_fr5_motion>/config/scene.yaml

Only our nodes talk to MoveIt (X-2): the control loop uses these services, never MoveIt directly.
- Static boxes from obstacles.yaml, added once move_group is up (former obstacle_loader, S-1). Never removable.
- /scene/attach_board (AttachBoard): picked board as a box under the tcp, contact_margin at its bottom (S-4).
  RULE S-11: a board added with add_box must be removed before attaching it.
- /scene/detach_board (DetachBoard): keep_as_obstacle -> fixed box stack_<n> with the attached shape (S-9).
- /scene/add_box (AddBox), /scene/remove_box (RemoveBox): dynamic boxes in base_link.
- After attach / add, the current robot state is checked (/check_state_validity): contacts are reported in the
  message (success stays true) so collision problems show up at once, not at the next plan.
- Single-threaded executor + async callbacks in a REENTRANT callback group (Q20-5: in the default mutually
  exclusive group an awaiting async service / timer callback blocks its own timers and client responses ->
  deadlock); one scene change at a time via the busy flag (others get "busy").
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from hk_fr5_interfaces.srv import AddBox, AttachBoard, DetachBoard, RemoveBox
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

# name -> type; values come only from config/scene.yaml (no defaults in code)
PARAMS = {
    "base_frame": Parameter.Type.STRING,
    "tcp_link": Parameter.Type.STRING,
    "group": Parameter.Type.STRING,
    "touch_links": Parameter.Type.STRING_ARRAY,
    "board_id": Parameter.Type.STRING,
    "stack_prefix": Parameter.Type.STRING,
    "contact_margin": Parameter.Type.DOUBLE,
    "load_static": Parameter.Type.BOOL,
    "obstacles_file": Parameter.Type.STRING,
    "wait_timeout": Parameter.Type.DOUBLE,
    "service_timeout": Parameter.Type.DOUBLE,
    "apply_service": Parameter.Type.STRING,
    "validity_service": Parameter.Type.STRING,
}


# ---- static boxes (former obstacle_loader, same rules) ---------------------------------------
def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _vector3(box, key, where):
    value = box.get(key)
    if not (isinstance(value, list) and len(value) == 3 and all(_number(v) for v in value)):
        raise ValueError(f"{where}: '{key}' must be a list of 3 numbers, got {value!r}")
    return [float(v) for v in value]


def load_boxes(path):
    """Return [(name, size, position, yaw)] or raise ValueError with a clear message."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    boxes = data.get("boxes") if isinstance(data, dict) else None
    if not isinstance(boxes, list) or not boxes:
        raise ValueError(f"{path}: 'boxes' must be a non-empty list")
    result, names = [], set()
    for i, box in enumerate(boxes):
        where = f"{path}: boxes[{i}]"
        if not isinstance(box, dict):
            raise ValueError(f"{where} must be a mapping")
        name = box.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{where}: 'name' must be a non-empty string")
        if name in names:
            raise ValueError(f"{where}: duplicate name '{name}'")
        names.add(name)
        where = f"{where} ({name})"
        size = _vector3(box, "size", where)
        if any(s <= 0.0 for s in size):
            raise ValueError(f"{where}: all sizes must be > 0, got {size}")
        position = _vector3(box, "position", where)
        yaw = box.get("yaw", 0.0)
        if not _number(yaw):
            raise ValueError(f"{where}: 'yaw' must be a number, got {yaw!r}")
        result.append((name, size, position, float(yaw)))
    return result


# ---- geometry --------------------------------------------------------------------------------
def yaw_quat(yaw):
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    x, y, z, w = q
    vq = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-x, -y, -z, w))
    return vq[:3]


def make_pose(position, quat):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat  # all 4 (Q-1)
    return pose


def box_object(name, frame, size, position, quat, operation=CollisionObject.ADD):
    obj = CollisionObject(id=name, operation=operation)
    obj.header.frame_id = frame
    if operation == CollisionObject.ADD:
        obj.pose = make_pose(position, quat)
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
        obj.primitive_poses = [make_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))]  # box centred on the pose
    return obj


def board_shape(req, default_margin):
    """(size, centre, quat) of the attached box in the tcp frame (decision-log 5n-1)."""
    margin = req.contact_margin or default_margin
    thick = req.height - margin
    return (req.length, req.width, thick), (req.dx, req.dy, thick / 2.0), yaw_quat(req.yaw), margin


def validate_attach(req, default_margin):
    for f in ("length", "width", "height", "yaw", "dx", "dy", "contact_margin"):
        if not math.isfinite(getattr(req, f)):
            return f"'{f}' is not a finite number"
    for f in ("length", "width", "height"):
        if getattr(req, f) <= 0.0:
            return f"'{f}' must be > 0, got {getattr(req, f)}"
    if req.contact_margin < 0.0:
        return f"'contact_margin' must be >= 0 (0 = default {default_margin}), got {req.contact_margin}"
    margin = req.contact_margin or default_margin
    if margin >= req.height:
        return f"contact_margin {margin} must be < height {req.height} (pass a smaller margin for thin boards)"
    return None


def validate_box(req):
    if not req.name:
        return "'name' must not be empty"
    values = list(req.size) + list(req.position) + [req.yaw]
    if not all(math.isfinite(v) for v in values):
        return "size / position / yaw must be finite numbers"
    if any(s <= 0.0 for s in req.size):
        return f"all sizes must be > 0, got {list(req.size)}"
    return None


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager", namespace="scene")
        self.cfg = {}
        for name, ptype in PARAMS.items():
            self.declare_parameter(name, ptype)
            try:
                self.cfg[name] = self.get_parameter(name).value
            except ParameterUninitializedException:
                raise RuntimeError(f"parameter '{name}' missing: start with "
                                   "--params-file <hk_fr5_motion share>/config/scene.yaml") from None
        c = self.cfg
        if not 0.0 < c["contact_margin"] < 0.05:
            raise RuntimeError(f"bad config: contact_margin must be in (0, 0.05) m, got {c['contact_margin']}")

        self.static = {}           # name -> (size, position, yaw), from obstacles.yaml
        self.static_loaded = False
        if c["load_static"]:
            path = c["obstacles_file"] or os.path.join(
                get_package_share_directory("hk_fr5_motion"), "config", "obstacles.yaml")
            try:
                boxes = load_boxes(path)  # bad yaml -> node fails at start (loud)
            except (OSError, ValueError, yaml.YAMLError) as e:
                raise RuntimeError(f"static obstacles: {e}") from None
            self.static = {n: (s, p, y) for n, s, p, y in boxes}
            self.static_path = path
        self.dynamic = {}          # name -> (size, centre) of every dynamic box, incl. stack_<n>
        self.attached = None       # dict(size, centre, quat, margin) while a board is attached
        self.stack_count = 0       # next stack_<n> index (monotonic: names are never reused)
        self._busy = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb = ReentrantCallbackGroup()  # Q20-5: async callbacks must not block each other's timers / responses
        self.apply = self.create_client(ApplyPlanningScene, c["apply_service"], callback_group=self.cb)
        self.validity = self.create_client(GetStateValidity, c["validity_service"], callback_group=self.cb)
        self.create_service(AttachBoard, "attach_board", self._attach, callback_group=self.cb)
        self.create_service(DetachBoard, "detach_board", self._detach, callback_group=self.cb)
        self.create_service(AddBox, "add_box", self._add_box, callback_group=self.cb)
        self.create_service(RemoveBox, "remove_box", self._remove_box, callback_group=self.cb)

        self._start = self.get_clock().now()
        if self.static:
            self.get_logger().info(f"waiting up to {c['wait_timeout']:.0f} s for {c['apply_service']} "
                                   f"(move_group) to add {len(self.static)} static boxes ...")
            self._static_timer = self.create_timer(0.5, self._load_static, callback_group=self.cb)
        else:
            self.get_logger().info("static obstacles disabled (load_static false)")

    # ---- MoveIt calls --------------------------------------------------------------------------
    async def _sleep(self, seconds):
        """Non-blocking sleep inside an async callback (timer-backed future)."""
        done = Future()
        timer = self.create_timer(seconds, lambda: done.done() or done.set_result(None), callback_group=self.cb)
        await done
        self.destroy_timer(timer)

    async def _call(self, client, request):
        """Service call with service_timeout; returns the response or None."""
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        waited = 0.0
        while not future.done() and waited < self.cfg["service_timeout"]:
            await self._sleep(0.01)
            waited += 0.01
        if not future.done():
            future.cancel()
            return None
        return future.result()

    async def _apply(self, scene):
        """Apply a planning scene diff. Returns an error message or None."""
        response = await self._call(self.apply, ApplyPlanningScene.Request(scene=scene))
        if response is None:
            return f"MoveIt not reachable ({self.cfg['apply_service']}, is move_group running?)"
        if not response.success:
            return "move_group rejected the planning scene update"
        return None

    async def _contacts(self):
        """'' if the current robot state is collision free, else a short contact list (debug aid, 5n-1)."""
        request = GetStateValidity.Request(group_name=self.cfg["group"])
        request.robot_state.is_diff = True  # = current state, incl. attached objects
        response = await self._call(self.validity, request)
        if response is None:
            return f" (state check skipped: {self.cfg['validity_service']} not reachable)"
        if response.valid:
            return ""
        pairs = sorted({tuple(sorted((x.contact_body_1, x.contact_body_2))) for x in response.contacts})
        text = ", ".join(f"{a} <-> {b}" for a, b in pairs[:4]) or "no contact details"
        self.get_logger().warn(f"robot state now IN COLLISION: {text}")
        return f"; WARNING: robot state now in collision: {text} (decision-log 5n-1, E-*)"

    def _world(self, objects):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        return scene

    async def _load_static(self):
        if self._busy:
            return
        waited = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if not self.apply.service_is_ready():
            if waited > self.cfg["wait_timeout"]:
                self.get_logger().error(f"static obstacles NOT added: {self.cfg['apply_service']} not available "
                                        f"after {self.cfg['wait_timeout']:.0f} s (is move_group running?)")
                self._static_timer.cancel()
            return
        self._static_timer.cancel()
        self._busy = True
        try:
            f = self.cfg["base_frame"]
            objects = [box_object(n, f, s, p, yaw_quat(y)) for n, (s, p, y) in self.static.items()]
            error = await self._apply(self._world(objects))
        finally:
            self._busy = False
        if error:
            self.get_logger().error(f"static obstacles NOT added: {error}")
            return
        for name, (size, position, yaw) in self.static.items():
            self.get_logger().info(f"added '{name}': size {size} m, centre {position} m, yaw {yaw:.3f} rad")
        self.get_logger().info(f"{len(self.static)} obstacles added to the planning scene from {self.static_path}")
        self.static_loaded = True

    # ---- services ------------------------------------------------------------------------------
    def _done(self, response, ok, message):
        response.success, response.message = ok, message
        if ok:  # rclpy fixes the severity per call site: one call per severity (Q20-1)
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    async def _attach(self, req, response):
        if self._busy:
            return self._done(response, False, "attach_board: scene busy with another change, retry")
        error = validate_attach(req, self.cfg["contact_margin"])
        if error:
            return self._done(response, False, f"attach_board: {error}")
        if self.attached is not None:
            return self._done(response, False, "attach_board: a board is already attached (detach it first)")
        size, centre, quat, margin = board_shape(req, self.cfg["contact_margin"])
        aco = AttachedCollisionObject(link_name=self.cfg["tcp_link"], touch_links=list(self.cfg["touch_links"]))
        aco.object = box_object(self.cfg["board_id"], self.cfg["tcp_link"], size, centre, quat)
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self._busy = True
        try:
            error = await self._apply(scene)
            if error:
                return self._done(response, False, f"attach_board: {error}")
            self.attached = {"size": size, "centre": centre, "quat": quat, "margin": margin}
            warning = await self._contacts()
        finally:
            self._busy = False
        return self._done(response, True,
                          f"attached '{self.cfg['board_id']}' to {self.cfg['tcp_link']}: "
                          f"{req.length * 1000:.1f} x {req.width * 1000:.1f} x {req.height * 1000:.1f} mm "
                          f"(modelled {size[2] * 1000:.1f} mm, margin {margin * 1000:.1f} mm), "
                          f"yaw {math.degrees(req.yaw):.1f} deg, offset ({req.dx * 1000:.1f}, {req.dy * 1000:.1f}) mm"
                          f"{warning}")

    def _board_in_base(self):
        """(position, quat) of the attached box centre in base_frame, from TF (base -> tcp)."""
        tf = self.tf_buffer.lookup_transform(self.cfg["base_frame"], self.cfg["tcp_link"], Time())
        t, r = tf.transform.translation, tf.transform.rotation
        q_tcp = (r.x, r.y, r.z, r.w)
        offset = quat_rotate(q_tcp, self.attached["centre"])
        return (t.x + offset[0], t.y + offset[1], t.z + offset[2]), quat_mul(q_tcp, self.attached["quat"])

    async def _detach(self, req, response):
        if self._busy:
            return self._done(response, False, "detach_board: scene busy with another change, retry")
        if self.attached is None:
            return self._done(response, False, "detach_board: no board attached")
        board, base = self.cfg["board_id"], self.cfg["base_frame"]
        objects = [box_object(board, base, None, None, None, CollisionObject.REMOVE)]  # MoveIt puts it in the world
        name = ""
        if req.keep_as_obstacle:
            try:
                position, quat = self._board_in_base()
            except TransformException as e:
                return self._done(response, False, f"detach_board: no tcp pose from TF ({e}); board still attached")
            name = f"{self.cfg['stack_prefix']}{self.stack_count}"
            objects.append(box_object(name, base, self.attached["size"], position, quat))  # S-9: same shape
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name=self.cfg["tcp_link"])
        aco.object = box_object(board, self.cfg["tcp_link"], None, None, None, CollisionObject.REMOVE)
        scene.robot_state.attached_collision_objects = [aco]
        scene.world.collision_objects = objects
        self._busy = True
        try:
            error = await self._apply(scene)
        finally:
            self._busy = False
        if error:
            return self._done(response, False, f"detach_board: {error}; board still attached")
        if name:
            self.dynamic[name] = (self.attached["size"], position)
            self.stack_count += 1
        self.attached = None
        response.name = name
        if name:
            return self._done(response, True, f"detached '{board}', kept as obstacle '{name}' at "
                                              f"({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) m")
        return self._done(response, True, f"detached '{board}' and removed it")

    async def _add_box(self, req, response):
        if self._busy:
            return self._done(response, False, "add_box: scene busy with another change, retry")
        error = validate_box(req)
        if error:
            return self._done(response, False, f"add_box: {error}")
        if req.name in self.static:
            return self._done(response, False, f"add_box: '{req.name}' is a static obstacle (obstacles.yaml)")
        if req.name == self.cfg["board_id"]:
            return self._done(response, False, f"add_box: '{req.name}' is reserved for the attached board")
        size, position = [float(v) for v in req.size], [float(v) for v in req.position]
        replaced = req.name in self.dynamic
        self._busy = True
        try:
            error = await self._apply(self._world(
                [box_object(req.name, self.cfg["base_frame"], size, position, yaw_quat(req.yaw))]))
            if error:
                return self._done(response, False, f"add_box: {error}")
            self.dynamic[req.name] = (size, position)
            warning = await self._contacts()
        finally:
            self._busy = False
        return self._done(response, True, f"{'replaced' if replaced else 'added'} box '{req.name}': size "
                                          f"{[round(v, 4) for v in size]} m, centre {[round(v, 4) for v in position]} m, "
                                          f"yaw {req.yaw:.3f} rad{warning}")

    async def _remove_box(self, req, response):
        if self._busy:
            return self._done(response, False, "remove_box: scene busy with another change, retry")
        if req.name in self.static:
            return self._done(response, False, f"remove_box: '{req.name}' is a static obstacle, never removed")
        if req.name == self.cfg["board_id"]:
            return self._done(response, False, "remove_box: the attached board is removed with detach_board")
        if req.name and req.name not in self.dynamic:
            return self._done(response, False, f"remove_box: unknown box '{req.name}' "
                                               f"(dynamic boxes: {sorted(self.dynamic) or 'none'})")
        names = [req.name] if req.name else sorted(self.dynamic)
        if not names:
            return self._done(response, True, "removed 0 boxes (no dynamic boxes)")
        self._busy = True
        try:
            error = await self._apply(self._world(
                [box_object(n, self.cfg["base_frame"], None, None, None, CollisionObject.REMOVE) for n in names]))
        finally:
            self._busy = False
        if error:
            return self._done(response, False, f"remove_box: {error}")
        for n in names:
            del self.dynamic[n]
        return self._done(response, True, f"removed {len(names)} box{'es' if len(names) != 1 else ''}: {', '.join(names)}")


def main():
    rclpy.init()
    try:
        node = SceneManager()
    except RuntimeError as e:
        print(f"[scene_manager] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)  # single-threaded executor; async callbacks interleave
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
