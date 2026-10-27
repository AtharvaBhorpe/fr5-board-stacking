"""Command / inspect the main controller from a shell (live runs). Run with `pixi run python3 tools/task_cli.py ...`.

Look (no effect on the robot):
    status                         controller state, current skill + phase, readiness, jobs, problem boards, last alerts
    world                          boards (id, variant, location, zone, pose, top) + zones (state, count, heights)
    list                           all jobs with counters
    wait JOB [TIMEOUT_S]           follow a job until DONE / CANCELLED (prints every change)

Jobs (queued; the controller decides when they run):
    transfer VARIANT|any TO [COUNT] [PRIO]          input boards of a variant -> zone TO (COUNT 0 = all)
    board BOARD_ID TO [PRIO]                        exactly this board -> TO (also an UNKWN board)
    process VARIANT|any ZONE [TO] [COUNT] [PRIO]    input board -> processing ZONE -> (after processing) TO
    clear FROM TO [COUNT] [PRIO]                    boards of zone FROM -> TO (not the fault zone)
    add k=v ...                                     any Job field (type, select_variant, select_board, from_zone, zone,
                                                    to, count, max_attempts, priority)
Control:
    pause [JOB] | resume [JOB] | stop | home | cancel JOB | prio JOB N | clear_queue
    reset_zone ZONE                the operator emptied ZONE: tell the world model (no robot motion)
Direct skill (debug; MOVES THE ROBOT, refused unless the controller is paused or idle; --force skips that check):
    skill pick board_id=b3 [contact_search=true] [velocity_scaling=0.1]
    skill place target=drop_off_a | skill place x=-0.3 y=0.4 yaw=1.57
    skill home | skill wait_clear [timeout=10] | skill release_safe [target=fault]
"""
import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from hk_fr5_task_interfaces.action import Home, Pick, Place, ReleaseSafe, WaitClear
from hk_fr5_task_interfaces.msg import Job, World
from hk_fr5_task_interfaces.srv import AddJob, Control, ListJobs

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
INT = ("count", "max_attempts", "priority")
SKILLS = {"pick": Pick, "place": Place, "home": Home, "wait_clear": WaitClear, "release_safe": ReleaseSafe}
FINAL = ("DONE", "CANCELLED")


def call(n, srv, name, req, timeout=10.0):
    c = n.create_client(srv, name)
    if not c.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f"{name} not available (task_main_controller running? same ROS_DOMAIN_ID?)")
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=timeout)
    if f.result() is None:
        raise SystemExit(f"{name}: no answer")
    return f.result()


def latest(n, typ, topic, timeout=3.0):
    box = {}
    sub = n.create_subscription(typ, topic, lambda m: box.update(m=m), LATCHED)
    end = time.time() + timeout
    while "m" not in box and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_subscription(sub)
    if "m" not in box:
        raise SystemExit(f"nothing on {topic} within {timeout:.0f} s")
    return box["m"]


def add(n, **f):
    j = Job(source="cli")
    for k, v in f.items():
        if k not in Job.get_fields_and_field_types() or k in ("id", "state"):
            raise SystemExit(f"unknown job field '{k}'")
        setattr(j, k, int(v) if k in INT else str(v))
    r = call(n, AddJob, "/task/add_job", AddJob.Request(job=j))
    print(f"{'OK' if r.success else 'REJECTED'} {r.id} {r.message}")


def control(n, cmd, job="", prio=0):
    r = call(n, Control, "/task/control", Control.Request(command=cmd, job_id=job, priority=int(prio)))
    print(f"{'OK' if r.success else 'REJECTED'} {cmd} {job} {r.message}")


def jobs(n):
    return call(n, ListJobs, "/task/list_jobs", ListJobs.Request()).jobs


def fmt_job(j):
    return (f"{j.id:4} {j.state:9} {j.type:10} variant={j.select_variant or 'any'} board={j.select_board or '-'} "
            f"from={j.from_zone or 'input'} zone={j.zone or '-'} to={j.to or '-'} count={j.count} prio={j.priority} | "
            f"att {j.attempts} delivered {j.delivered} fault {j.to_fault} lost {j.lost} skipped {j.skipped} | {j.message}")


def opt(a, i, default):
    return a[i] if len(a) > i else default


def var(v):
    return "" if v in ("any", "") else v


def skill(n, name, kv, force):
    if name not in SKILLS:
        raise SystemExit(f"unknown skill '{name}' ({' | '.join(SKILLS)})")
    if not force:
        st = json.loads(latest(n, String, "/task/status").data)
        busy = [j["id"] for j in st.get("jobs", []) if j["state"] in ("RUNNING", "WAITING", "QUEUED")]
        if not st.get("paused") and (busy or st.get("current")):
            raise SystemExit(f"controller is working ({busy or st.get('current')}): 'pause' first, or --force")
    typ = SKILLS[name]
    g = typ.Goal()
    for item in kv:
        k, _, v = item.partition("=")
        ft = g.get_fields_and_field_types().get(k)
        if ft is None:
            raise SystemExit(f"{name}: unknown goal field '{k}' ({', '.join(g.get_fields_and_field_types())})")
        setattr(g, k, v.lower() in ("1", "true", "yes") if ft == "boolean" else float(v) if ft == "double" else v)
    c = ActionClient(n, typ, f"/task/skills/{name}")
    if not c.wait_for_server(timeout_sec=5.0):
        raise SystemExit(f"/task/skills/{name} not available (skill_server running?)")
    print(f"sending {name} {g}")
    f = c.send_goal_async(g, feedback_callback=lambda fb: print(f"  phase {getattr(fb.feedback, 'phase', fb.feedback)}"))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    gh = f.result()
    if gh is None or not gh.accepted:
        raise SystemExit("goal refused (another skill running?)")
    rf = gh.get_result_async()
    try:
        while not rf.done():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Ctrl+C: cancelling the skill (smooth stop)")
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(n, cf, timeout_sec=5.0)
        rclpy.spin_until_future_complete(n, rf, timeout_sec=10.0)
    r = rf.result().result if rf.done() else None
    print(f"result: {'no result' if r is None else (r.success, r.code.value, r.message)}")


def main():
    a = sys.argv[1:]
    force = "--force" in a
    a = [x for x in a if x != "--force"]
    if not a or a[0] in ("-h", "--help", "help"):
        raise SystemExit(__doc__)
    rclpy.init()
    n = rclpy.create_node("task_cli")
    c = a[0]
    try:
        if c == "status":
            st = json.loads(latest(n, String, "/task/status").data)
            cur = st.get("current") or {}
            print(f"state {st.get('state')} paused {st.get('paused')} halted '{st.get('halted', '')}' | skills_ready "
                  f"{st.get('skills_ready')} motion_ready {st.get('motion_ready')} world_ok {st.get('world_ok')} "
                  f"world_age {st.get('world_age')} | tcp {st.get('tcp')}")
            print(f"current {cur or '-'} phase {st.get('phase') or '-'} | problem boards {st.get('problem_boards')}")
            for j in st.get("jobs", []):
                print(f"  job {j.get('id')} {j.get('state')} {j.get('type')} -> {j.get('to')} delivered {j.get('delivered')} "
                      f"fault {j.get('to_fault')} lost {j.get('lost')} skipped {j.get('skipped')} | {j.get('message')}")
            for e in st.get("events", [])[-10:]:
                print(f"  event {e.get('level', '')} {e.get('text', e)}")
        elif c == "world":
            w = latest(n, World, "/task/world")
            print(f"vision_ok {w.vision_ok} age {w.vision_age:.2f} s | held '{w.held_board_id}' gripper {w.gripper_state} "
                  f"| obstacles {[(o.label, round(o.x, 3), round(o.y, 3), o.in_work_area) for o in w.obstacles]}")
            for z in w.zones:
                print(f"  zone {z.name:13} {z.kind:10} {z.state:10} count {z.stack_count} top {z.stack_top_z:.4f} base "
                      f"{z.base_z:.4f} board '{z.board_id}' at ({z.x:.3f}, {z.y:.3f}) yaw {z.yaw:.2f} "
                      f"{z.length:.3f} x {z.width:.3f}{' FIXED' if z.fixed else ''} age {z.age:.1f} {z.message}")
            for b in w.boards:
                print(f"  board {b.id:4} {b.variant:7} label {b.label:6} {b.location:5} {b.zone:13} ({b.x:.3f}, {b.y:.3f}) "
                      f"yaw {b.yaw:.2f} {b.length:.3f} x {b.width:.3f} top {b.top_z:.4f} size_ok {b.size_ok} "
                      f"processed {b.processed} age {b.age:.1f}")
        elif c == "list":
            for j in jobs(n):
                print(fmt_job(j))
        elif c == "wait":
            jid, end, last = a[1], time.time() + float(opt(a, 2, 900)), None
            while time.time() < end:
                j = next((j for j in jobs(n) if j.id == jid), None)
                if j is None:
                    raise SystemExit(f"unknown job {jid}")
                if fmt_job(j) != last:
                    last = fmt_job(j); print(time.strftime("%H:%M:%S"), last)
                if j.state in FINAL:
                    break
                time.sleep(1.0)
        elif c == "transfer":
            add(n, type="transfer", select_variant=var(a[1]), to=a[2], count=opt(a, 3, 0), priority=opt(a, 4, 0))
        elif c == "board":
            add(n, type="transfer", select_board=a[1], to=a[2], count=1, priority=opt(a, 3, 0))
        elif c == "process":
            add(n, type="process", select_variant=var(a[1]), zone=a[2], to=opt(a, 3, ""), count=opt(a, 4, 1),
                priority=opt(a, 5, 0))
        elif c == "clear":
            add(n, type="clear_zone", from_zone=a[1], to=a[2], count=opt(a, 3, 0), priority=opt(a, 4, 0))
        elif c == "add":
            add(n, **dict(kv.partition("=")[::2] for kv in a[1:]))
        elif c in ("pause", "resume"):
            control(n, c, opt(a, 1, ""))
        elif c in ("stop", "home", "clear_queue"):
            control(n, c)
        elif c == "cancel":
            control(n, "cancel_job", a[1])
        elif c == "prio":
            control(n, "set_priority", a[1], a[2])
        elif c == "control":                                  # raw form: control CMD [JOB] [PRIO]
            control(n, a[1], opt(a, 2, ""), opt(a, 3, 0))
        elif c == "reset_zone":
            pub = n.create_publisher(String, "/task/events", 10)
            end = time.time() + 2.0
            while pub.get_subscription_count() == 0 and time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.05)
            pub.publish(String(data=json.dumps({"type": "reset_zone", "zone": a[1]})))
            rclpy.spin_once(n, timeout_sec=0.3)
            print(f"sent reset_zone {a[1]} ({pub.get_subscription_count()} subscriber(s))")
        elif c == "skill":
            skill(n, a[1], a[2:], force)
        else:
            raise SystemExit(f"unknown command '{c}'\n{__doc__}")
    except IndexError:
        raise SystemExit(f"missing argument for '{c}'\n{__doc__}")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
