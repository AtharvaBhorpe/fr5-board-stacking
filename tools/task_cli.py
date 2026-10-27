"""Command the main controller from a shell (live runs).

    pixi run python3 tools/task_cli.py add type=transfer select_variant=VARa to=drop_off_a count=1 [priority=5]
    pixi run python3 tools/task_cli.py control pause|resume|stop|home|clear_queue
    pixi run python3 tools/task_cli.py control cancel_job j3 | control set_priority j3 10
    pixi run python3 tools/task_cli.py list
"""
import sys

import rclpy

from hk_fr5_task_interfaces.msg import Job
from hk_fr5_task_interfaces.srv import AddJob, Control, ListJobs

INT = ("count", "max_attempts", "priority")


def call(n, srv, name, req, timeout=10.0):
    c = n.create_client(srv, name)
    if not c.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f"{name} not available (task_main_controller running? same ROS_DOMAIN_ID?)")
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=timeout)
    if f.result() is None:
        raise SystemExit(f"{name}: no answer")
    return f.result()


def main():
    a = sys.argv[1:]
    if not a or a[0] not in ("add", "control", "list"):
        raise SystemExit(__doc__)
    rclpy.init()
    n = rclpy.create_node("task_cli")
    try:
        if a[0] == "add":
            j = Job(source="cli")
            for kv in a[1:]:
                k, _, v = kv.partition("=")
                if k not in Job.get_fields_and_field_types() or k in ("id", "state"):
                    raise SystemExit(f"unknown job field '{k}'")
                setattr(j, k, int(v) if k in INT else v)
            r = call(n, AddJob, "/task/add_job", AddJob.Request(job=j))
            print(f"{'OK' if r.success else 'REJECTED'} {r.id} {r.message}")
        elif a[0] == "control":
            if len(a) < 2:
                raise SystemExit(__doc__)
            r = call(n, Control, "/task/control", Control.Request(command=a[1], job_id=a[2] if len(a) > 2 else "",
                                                                 priority=int(a[3]) if len(a) > 3 else 0))
            print(f"{'OK' if r.success else 'REJECTED'} {r.message}")
        else:
            r = call(n, ListJobs, "/task/list_jobs", ListJobs.Request())
            for j in r.jobs:
                print(f"{j.id:4} {j.state:9} {j.type:10} variant={j.select_variant or 'any'} board={j.select_board or '-'} "
                      f"from={j.from_zone or 'input'} zone={j.zone or '-'} to={j.to} count={j.count} prio={j.priority} | "
                      f"delivered {j.delivered} fault {j.to_fault} lost {j.lost} skipped {j.skipped} | {j.message}")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
