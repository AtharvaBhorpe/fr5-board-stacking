"""Helpers for scene tests: launch bringup headless, read the planning scene."""
import os, signal, subprocess, time
from testenv import SCENE_CFG, SHELL, MOCK, GRIPPER_CFG, LOG_DIR  # noqa: F401
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
def launch(*args, log="launch.log"):
    return subprocess.Popen(["ros2", "launch", "hk_fr5_moveit_config", "bringup.launch.py", "use_rviz:=false", *args],
                            stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
def node(*args, log="scene.log"):
    return subprocess.Popen(["ros2", "run", "hk_fr5_motion", "scene_manager", "--ros-args", "--params-file", SCENE_CFG, *args],
                            stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
def stop(p, wait=8):
    try: os.killpg(p.pid, signal.SIGINT)
    except ProcessLookupError: return
    t = time.time()
    while p.poll() is None and time.time() - t < wait: time.sleep(0.2)
    try: os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError: pass
def call(n, client, req, timeout=10.0):
    assert client.wait_for_service(timeout_sec=timeout), client.srv_name
    f = client.call_async(req); end = time.time() + timeout
    while not f.done() and time.time() < end: rclpy.spin_once(n, timeout_sec=0.02)
    return f.result()
def scene(n, client):
    comp = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    return call(n, client, GetPlanningScene.Request(components=PlanningSceneComponents(components=comp))).scene
def world(sc):
    """{id: (dims, pose xyz, quat xyzw)} rounded."""
    out = {}
    for o in sc.world.collision_objects:
        p = o.pose; pp = o.primitive_poses[0] if o.primitive_poses else None
        out[o.id] = (tuple(round(v, 6) for v in o.primitives[0].dimensions), o.header.frame_id,
                     tuple(round(v, 6) for v in (p.position.x, p.position.y, p.position.z)),
                     tuple(round(v, 6) for v in (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)),
                     None if pp is None else tuple(round(v, 6) for v in (pp.position.x, pp.position.y, pp.position.z, pp.orientation.w)))
    return out
