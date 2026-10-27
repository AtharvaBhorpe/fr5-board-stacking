"""scene_manager services against the sim bringup (T20b): attach / detach (keep) / add / remove + rejections.
The bringup's own scene_manager is used (T20d)."""
import time, rclpy
from moveit_msgs.srv import GetPlanningScene
from hk_fr5_interfaces.srv import AttachBoard, DetachBoard, AddBox, RemoveBox, GetState
from scene_lib import launch, stop, call, scene, world

ok = 0
def check(name, cond, extra=""):
    global ok
    print(("PASS " if cond else "FAIL ") + name + (f"  [{extra}]" if extra else ""), flush=True); ok += bool(cond)
    if not cond: raise SystemExit(1)

rclpy.init(); n = rclpy.create_node("scene_smoke"); gps = n.create_client(GetPlanningScene, "/get_planning_scene")
c = {k: n.create_client(t, f"/scene/{k}") for k, t in (("attach_board", AttachBoard), ("detach_board", DetachBoard),
                                                       ("add_box", AddBox), ("remove_box", RemoveBox))}
L = launch("use_gripper:=false", log="scene_smoke_launch.log"); time.sleep(18)
gs = n.create_client(GetState, "/motion/get_state")
try:
    home_z = call(n, gs, GetState.Request()).tcp_pose.pose.position.z     # from the URDF (tool_geometry tcp_z), not fixed
    r = call(n, c["attach_board"], AttachBoard.Request(length=0.27, width=0.20, height=0.006))
    a = scene(n, gps).robot_state.attached_collision_objects
    check("attach 270 x 200 x 6 mm at home", r.success, r.message)
    check("MoveIt: 'board' on tcp, touch_links gripper_link + tcp, 4 mm box centred 2 mm below tcp",
          len(a) == 1 and a[0].object.id == "board" and a[0].link_name == "tcp" and list(a[0].touch_links) == ["gripper_link", "tcp"]
          and [round(v, 4) for v in a[0].object.primitives[0].dimensions] == [0.27, 0.2, 0.004]
          and abs(a[0].object.pose.position.z - 0.002) < 1e-6)
    r = call(n, c["attach_board"], AttachBoard.Request(length=0.27, width=0.20, height=0.006))
    check("2nd attach rejected", not r.success and "already attached" in r.message, r.message)
    r = call(n, c["detach_board"], DetachBoard.Request(keep_as_obstacle=True)); sc = scene(n, gps); w = world(sc)
    check("detach keep -> stack_0 (name field), nothing attached, no 'board' in the world",
          r.success and r.name == "stack_0" and not sc.robot_state.attached_collision_objects and "board" not in w and "stack_0" in w, r.message)
    check(f"stack_0 centre 2 mm below the home tcp (tcp z {home_z:.4f}), 4 mm thick",
          abs(w["stack_0"][2][2] - (home_z - 0.002)) < 5e-4 and w["stack_0"][0][2] == 0.004, f"centre z {w['stack_0'][2][2]:.4f}")
    r = call(n, c["add_box"], AddBox.Request(name="pick_board", size=[0.27, 0.2, 0.006], position=[-0.4, -0.4, -0.042], yaw=0.3))
    check("add_box pick_board", r.success and "WARNING" not in r.message, r.message)
    r = call(n, c["add_box"], AddBox.Request(name="table", size=[1.0, 1.0, 1.0], position=[0.0, 0.0, 0.0]))
    check("add_box with a static name rejected", not r.success, r.message)
    r = call(n, c["add_box"], AddBox.Request(name="board", size=[0.1, 0.1, 0.1], position=[0.0, 0.5, 0.5]))
    check("add_box named 'board' rejected", not r.success, r.message)
    r = call(n, c["add_box"], AddBox.Request(name="bad", size=[0.0, 0.1, 0.1], position=[0.0, 0.5, 0.5]))
    check("add_box with size 0 rejected", not r.success, r.message)
    r = call(n, c["add_box"], AddBox.Request(name="probe", size=[0.05, 0.05, 0.05], position=[-0.497, -0.102, 0.364]))
    check("box into the gripper: success + collision WARNING naming gripper_link", r.success and "WARNING" in r.message
          and "gripper_link" in r.message, r.message[-90:])
    r = call(n, c["remove_box"], RemoveBox.Request(name="table"))
    check("remove a static box rejected", not r.success, r.message)
    r = call(n, c["remove_box"], RemoveBox.Request(name="nothere"))
    check("remove unknown box rejected, lists dynamic boxes", not r.success and "pick_board" in r.message, r.message)
    r = call(n, c["remove_box"], RemoveBox.Request(name="")); w = world(scene(n, gps))
    check("remove '' -> 3 dynamic boxes, only the 4 static ones left", r.success and "3 boxes" in r.message
          and sorted(w) == ["camera_post", "floor_guard", "profile", "table"], r.message)
    r = call(n, c["detach_board"], DetachBoard.Request())
    check("detach with nothing attached rejected", not r.success, r.message)
    for req, what in ((AttachBoard.Request(length=0.27, width=0.2, height=0.006, contact_margin=0.01), "margin >= height"),
                      (AttachBoard.Request(length=0.27, width=0.0, height=0.006), "width 0"),
                      (AttachBoard.Request(length=0.27, width=0.2, height=0.006, contact_margin=-0.001), "negative margin")):
        r = call(n, c["attach_board"], req)
        check(f"attach with {what} rejected", not r.success, r.message)
finally:
    stop(L, wait=12)
    print(f"{ok} checks passed")
    n.destroy_node(); rclpy.shutdown()
