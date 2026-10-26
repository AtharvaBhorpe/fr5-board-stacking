"""motion_shell: command shell for manual tests of /motion/* (sim and real robot).

    pixi run shell        (next to `pixi run sim` or `pixi run robot`)

Input in mm / deg (sent as m / rad). Uses only the public API: /motion/move_to, /motion/move_delta,
/motion/get_state, /gripper/grip, /gripper/release, /gripper/state (step 20e) and
/scene/attach_board, detach_board, add_box, remove_box (T20e).
Ctrl+C while a goal runs = cancel (smooth stop / gripper cancel). Type `help` for commands.
"""
import math
import readline  # noqa: F401  (line editing + history for input())
import signal
import threading
import time

import rclpy
from hk_fr5_interfaces.action import Grip, MoveDelta, MoveTo, Release
from hk_fr5_interfaces.msg import GripperState
from hk_fr5_interfaces.srv import AddBox, AttachBoard, DetachBoard, GetState, RemoveBox
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions

from hk_fr5_motion.motion_controller import quat_to_rpy

CODES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
PLANNERS = {"ompl": "", "ptp": "PTP", "lin": "LIN"}
AXES = {"x": 0, "y": 1, "z": 2, "roll": 3, "pitch": 4, "yaw": 5}
HELP = """\
units: mm and deg (sent as m / rad). '.' = keep the current value of that field.
  state | s                       joints (deg, rad), tcp (mm), rpy (deg)
  status                          servers available, state ok / age
  to X Y Z [YAW]                  move_to, tool pointing down (no YAW: keep current yaw)
  to X Y Z ROLL PITCH YAW         move_to, full orientation
  home | named NAME               move_to an SRDF state (ready = home, zero)
  d DX DY DZ [DROLL DPITCH DYAW] [tcp|base]   move_delta (frame default: `frame` setting)
  jog +z [MM] | jog -yaw [DEG]    move_delta along / about one axis (step / rstep, frame setting)
  v 0.1 | planner ompl|ptp|lin | plan on|off | frame tcp|base | step MM | rstep DEG | set
  grip [TIMEOUT_S] | release [BLOW_S] | gstate   gripper (s; omitted / 0 = gripper.yaml default)
  attach L W H [YAW DX DY MARGIN]  board under the cup (mm / deg; MARGIN 0 = default 2 mm). Rules: only after vacuum,
                                  remove its pick-zone box first (S-11), lift with `d` (Pilz, S-12)
  detach [keep]                   remove the board, or keep it as obstacle stack_<n>
  box add NAME SX SY SZ X Y Z [YAW] | box rm [NAME]   dynamic box in base_link (mm / deg); rm without NAME = all
  history | help | quit
  Ctrl+C while moving = cancel (smooth stop); during grip / release = cancel (pump / blow-off off)
TCP frame: +z = out of the cup (DOWN when the tool points down). base frame: +z = up."""


class Shell:
    def __init__(self, ns="/motion", gripper_ns="/gripper"):
        self.node = rclpy.create_node("motion_shell")
        self.move_to = ActionClient(self.node, MoveTo, f"{ns}/move_to")
        self.move_delta = ActionClient(self.node, MoveDelta, f"{ns}/move_delta")
        self.get_state = self.node.create_client(GetState, f"{ns}/get_state")
        self.grip = ActionClient(self.node, Grip, f"{gripper_ns}/grip")
        self.release = ActionClient(self.node, Release, f"{gripper_ns}/release")
        self.gripper_state, self.gripper_stamp = None, 0.0  # latest /gripper/state + arrival time
        self.scene = {name: self.node.create_client(srv, f"/scene/{name}") for name, srv in
                      (("attach_board", AttachBoard), ("detach_board", DetachBoard),
                       ("add_box", AddBox), ("remove_box", RemoveBox))}
        self.node.create_subscription(GripperState, f"{gripper_ns}/state", self._on_gripper_state,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()
        self.scale, self.planner, self.plan_only = 0.10, "ompl", False
        self.frame, self.step, self.rstep = "tcp", 10.0, 5.0
        self.history = []

    def _on_gripper_state(self, msg):
        self.gripper_state, self.gripper_stamp = msg, time.time()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # shutdown
            pass

    @staticmethod
    def _wait(future, timeout):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.005)
        return future.result() if future.done() else None

    # ---- state -------------------------------------------------------------------------------
    def state(self):
        if not self.get_state.wait_for_service(timeout_sec=2.0):
            return None
        return self._wait(self.get_state.call_async(GetState.Request()), 3.0)

    def current(self):
        """[x, y, z (mm), roll, pitch, yaw (deg)] or None."""
        s = self.state()
        if s is None or not s.success:
            return None
        p = s.tcp_pose.pose.position
        return [p.x * 1000, p.y * 1000, p.z * 1000] + [math.degrees(a) for a in (s.roll, s.pitch, s.yaw)]

    def print_state(self):
        s = self.state()
        if s is None:
            print("✘ /motion/get_state not available (is the bringup running?)")
            return
        if not s.success:
            print(f"✘ state invalid: {s.message}")
            return
        print(f"  {s.message}")
        for name, q in zip(s.joints.name, s.joints.position):
            print(f"  {name}  {math.degrees(q):9.3f} deg  {q:9.5f} rad")
        p = s.tcp_pose.pose.position
        o = s.tcp_pose.pose.orientation
        print(f"  tcp  x {p.x * 1000:9.2f}  y {p.y * 1000:9.2f}  z {p.z * 1000:9.2f} mm   (base_link)")
        print(f"  rpy  {math.degrees(s.roll):8.2f} {math.degrees(s.pitch):8.2f} {math.degrees(s.yaw):8.2f} deg"
              f"   quat ({o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f})")

    def status(self):
        ok = lambda b: "●" if b else "○ (missing)"
        print(f"  move_to {ok(self.move_to.wait_for_server(timeout_sec=1.0))}   "
              f"move_delta {ok(self.move_delta.wait_for_server(timeout_sec=1.0))}   "
              f"get_state {ok(self.get_state.wait_for_service(timeout_sec=1.0))}")
        print(f"  gripper {'● (grip, release, state)' if self.gripper_up() else '○ not running (use_gripper:=false?)'}   "
              f"scene {'● (attach, detach, box)' if self.scene_up() else '○ not running'}")
        s = self.state()
        print(f"  state: {'OK' if s is not None and s.success else 'INVALID'}  "
              f"{'' if s is None else s.message}")
        self.print_settings()

    def print_settings(self):
        print(f"  settings: v {self.scale:.2f} | planner {self.planner} | plan_only {'on' if self.plan_only else 'off'}"
              f" | frame {self.frame} | step {self.step:g} mm | rstep {self.rstep:g} deg")

    # ---- goals -------------------------------------------------------------------------------
    def run(self, client, goal, label, describe=None, missing="is the bringup running?"):
        """Send a goal, print phase feedback, Ctrl+C = cancel; describe(result) -> extra text (default: tcp pose)."""
        if not client.wait_for_server(timeout_sec=2.0):
            print(f"✘ action server not available ({missing})")
            return
        print(f"→ {label}")
        t0 = time.time()
        # only feedback with a `phase` is printed (grip feedback = elapsed time every poll, too chatty)
        feedback = lambda m: hasattr(m.feedback, "phase") and print(f"   … {m.feedback.phase} ({time.time() - t0:.2f} s)")
        handle = self._wait(client.send_goal_async(goal, feedback_callback=feedback), 5.0)
        if handle is None or not handle.accepted:
            print("✘ goal not accepted")
            return
        result = handle.get_result_async()
        try:
            gone = self._wait_result(client, result)
        except KeyboardInterrupt:
            print("   ■ cancel requested ...")
            handle.cancel_goal_async()
            gone = self._wait_result(client, result)
        if gone:
            print("✘ action server disappeared while the goal was running (node stopped or crashed?)")
            self.history.append(f"{time.strftime('%H:%M:%S')} {label} -> ✘ server disappeared")
            return
        r = result.result().result
        extra = (describe or self._describe_pose)(r)
        line = (f"{'✔' if r.success else '✘'} {r.message} (code {r.error_code.val}) | "
                f"{extra + ' | ' if extra else ''}{time.time() - t0:.2f} s")
        print(line)
        self.history.append(f"{time.strftime('%H:%M:%S')} {label} -> {line}")

    @staticmethod
    def _wait_result(client, result):
        """Wait for a goal result; True if the action server disappeared meanwhile (node stopped / crashed)."""
        next_check = time.time() + 0.5
        while not result.done():
            time.sleep(0.01)
            if time.time() >= next_check:
                if not client.server_is_ready():
                    return True
                next_check = time.time() + 0.5
        return False

    @staticmethod
    def _describe_pose(r):
        p = r.final_tcp_pose.pose.position
        o = r.final_tcp_pose.pose.orientation
        rpy = [math.degrees(a) for a in quat_to_rpy(o.x, o.y, o.z, o.w)] if (o.x or o.y or o.z or o.w) else [0, 0, 0]
        return (f"final ({p.x * 1000:.1f}, {p.y * 1000:.1f}, {p.z * 1000:.1f}) mm, "
                f"rpy ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}) deg")

    def _values(self, args, n_expected, current_index_offset=0):
        """Parse numbers; '.' -> current value (mm / deg)."""
        cur = None
        out = []
        for i, a in enumerate(args):
            if a == ".":
                cur = cur or self.current()
                if cur is None:
                    raise ValueError("no valid current state for '.'")
                out.append(cur[i + current_index_offset])
            else:
                out.append(float(a))
        return out

    def cmd_to(self, args):
        if len(args) not in (3, 4, 6):
            raise ValueError("to X Y Z [YAW]  or  to X Y Z ROLL PITCH YAW")
        v = self._values(args[:3], 3)
        g = MoveTo.Goal(x=v[0] / 1000, y=v[1] / 1000, z=v[2] / 1000, velocity_scaling=self.scale,
                        planner=PLANNERS[self.planner], plan_only=self.plan_only)
        if len(args) == 6:
            r = self._values(args[3:], 3, current_index_offset=3)
            g.mode, (g.roll, g.pitch, g.yaw) = MoveTo.Goal.MODE_FULL, (math.radians(x) for x in r)
            orient = f"rpy ({r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f}) deg full"
        else:
            if len(args) == 4 and args[3] != ".":
                yaw = float(args[3])
            else:
                cur = self.current()
                if cur is None:
                    raise ValueError("no valid current state to keep the yaw")
                yaw = cur[5]
            g.mode, g.yaw = MoveTo.Goal.MODE_DOWN_YAW, math.radians(yaw)
            orient = f"tool down, yaw {yaw:.1f} deg"
        self.run(self.move_to, g, f"move_to ({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) mm, {orient}, {self._opts()}")

    def cmd_named(self, name):
        g = MoveTo.Goal(named_target=name, velocity_scaling=self.scale, planner=PLANNERS[self.planner],
                        plan_only=self.plan_only)
        self.run(self.move_to, g, f"move_to named '{name}', {self._opts()}")

    def cmd_delta(self, args, frame=None):
        frame = frame or self.frame
        words = [a for a in args if a in ("tcp", "base")]
        nums = [a for a in args if a not in ("tcp", "base")]
        if words:
            frame = words[-1]
        if len(nums) not in (3, 6):
            raise ValueError("d DX DY DZ [DROLL DPITCH DYAW] [tcp|base]")
        v = [float(a) for a in nums] + [0.0] * (6 - len(nums))
        g = MoveDelta.Goal(dx=v[0] / 1000, dy=v[1] / 1000, dz=v[2] / 1000,
                           droll=math.radians(v[3]), dpitch=math.radians(v[4]), dyaw=math.radians(v[5]),
                           frame=MoveDelta.Goal.FRAME_BASE if frame == "base" else MoveDelta.Goal.FRAME_TCP,
                           velocity_scaling=self.scale, plan_only=self.plan_only)
        self.run(self.move_delta, g, f"move_delta {frame} d ({v[0]:g}, {v[1]:g}, {v[2]:g}) mm "
                                     f"drpy ({v[3]:g}, {v[4]:g}, {v[5]:g}) deg, v {self.scale:.2f}"
                                     f"{', plan_only' if self.plan_only else ''}")

    def cmd_jog(self, args):
        if not args or args[0][0] not in "+-" or args[0][1:] not in AXES:
            raise ValueError("jog +z [MM] | jog -yaw [DEG]   (axes: x y z roll pitch yaw)")
        axis = AXES[args[0][1:]]
        amount = float(args[1]) if len(args) > 1 else (self.step if axis < 3 else self.rstep)
        v = [0.0] * 6
        v[axis] = amount if args[0][0] == "+" else -amount
        self.cmd_delta([str(x) for x in v])

    # ---- gripper -----------------------------------------------------------------------------
    GRIPPER_MISSING = "gripper node not running: bringup started with use_gripper:=false?"

    def gripper_up(self):
        """True if the gripper node is there (its grip action server). Not there -> drop the cached
        state, so a stopped gripper node never shows old data."""
        if self.grip.wait_for_server(timeout_sec=1.0):
            return True
        self.gripper_state = None
        return False

    @staticmethod
    def _seconds(args, usage):
        if len(args) > 1:
            raise ValueError(usage)
        value = float(args[0]) if args else 0.0
        if value < 0.0:
            raise ValueError(f"{usage}  (seconds >= 0, 0 = default)")
        return value

    def cmd_grip(self, args):
        timeout = self._seconds(args, "grip [TIMEOUT_S]")
        if not self.gripper_up():
            print(f"✘ {self.GRIPPER_MISSING}")
            return
        ttv = lambda r: f"time_to_vacuum {r.time_to_vacuum:.2f} s" if r.time_to_vacuum >= 0 else ""
        self.run(self.grip, Grip.Goal(timeout=timeout),
                 f"grip, timeout {f'{timeout:g} s' if timeout else 'default'}", describe=ttv,
                 missing=self.GRIPPER_MISSING)

    def cmd_release(self, args):
        blow = self._seconds(args, "release [BLOW_S]")
        if not self.gripper_up():
            print(f"✘ {self.GRIPPER_MISSING}")
            return
        self.run(self.release, Release.Goal(blow_off_time=blow),
                 f"release, blow-off {f'{blow:g} s' if blow else 'default'}", describe=lambda r: "",
                 missing=self.GRIPPER_MISSING)

    def print_gripper(self):
        if not self.gripper_up():
            print(f"✘ {self.GRIPPER_MISSING}")
            return
        g, end = self.gripper_state, time.monotonic() + 1.0
        while g is None and time.monotonic() < end:  # right after shell start the latched state can lag
            time.sleep(0.05)
            g = self.gripper_state
        if g is None:
            print("✘ gripper node is up but no /gripper/state received within 1 s, try again")
            return
        onoff = lambda b: "on " if b else "off"
        print(f"  gripper {g.state}  ({time.time() - self.gripper_stamp:.1f} s ago)  {g.message}")
        print(f"  vacuum pump {onoff(g.vacuum_on)} | blow-off {onoff(g.blow_off_on)} | "
              f"vacuum {'achieved' if g.vacuum_achieved else 'not achieved'}")

    # ---- scene -------------------------------------------------------------------------------
    SCENE_MISSING = "scene_manager not running (is the bringup running?)"

    def scene_up(self):
        return self.scene["attach_board"].wait_for_service(timeout_sec=1.0)

    def _scene_call(self, name, request, label):
        if not self.scene[name].wait_for_service(timeout_sec=1.0):
            print(f"✘ {self.SCENE_MISSING}")
            return
        r = self._wait(self.scene[name].call_async(request), 10.0)
        if r is None:
            print(f"✘ {label}: no answer from /scene/{name} within 10 s")
            return
        line = f"{'✔' if r.success else '✘'} {r.message}"
        print(line)
        self.history.append(f"{time.strftime('%H:%M:%S')} {label} -> {line}")

    def cmd_attach(self, args):
        usage = "attach L W H [YAW DX DY MARGIN]   (mm / deg)"
        if len(args) not in (3, 4, 6, 7):
            raise ValueError(usage)
        v = [float(a) for a in args] + [0.0] * (7 - len(args))
        req = AttachBoard.Request(length=v[0] / 1000, width=v[1] / 1000, height=v[2] / 1000, yaw=math.radians(v[3]),
                                  dx=v[4] / 1000, dy=v[5] / 1000, contact_margin=v[6] / 1000)
        self._scene_call("attach_board", req, f"attach {v[0]:g} x {v[1]:g} x {v[2]:g} mm")

    def cmd_detach(self, args):
        if args not in ([], ["keep"]):
            raise ValueError("detach [keep]")
        self._scene_call("detach_board", DetachBoard.Request(keep_as_obstacle=bool(args)),
                         f"detach{' keep' if args else ''}")

    def cmd_box(self, args):
        usage = "box add NAME SX SY SZ X Y Z [YAW]  |  box rm [NAME]   (mm / deg, base_link)"
        if args and args[0] == "add" and len(args) in (8, 9):
            v = [float(a) for a in args[2:]] + [0.0] * (9 - len(args))
            req = AddBox.Request(name=args[1], size=[x / 1000 for x in v[0:3]], position=[x / 1000 for x in v[3:6]],
                                 yaw=math.radians(v[6]))
            self._scene_call("add_box", req, f"box add {args[1]}")
        elif args and args[0] == "rm" and len(args) in (1, 2):
            name = args[1] if len(args) == 2 else ""
            self._scene_call("remove_box", RemoveBox.Request(name=name), f"box rm {name or '(all dynamic)'}")
        else:
            raise ValueError(usage)

    def _opts(self):
        return f"planner {self.planner}, v {self.scale:.2f}{', plan_only' if self.plan_only else ''}"

    # ---- loop --------------------------------------------------------------------------------
    def handle(self, line):
        args = line.split()
        if not args:
            return True
        c, a = args[0].lower(), args[1:]
        if c in ("quit", "exit", "q"):
            return False
        if c == "help":
            print(HELP)
        elif c in ("state", "s"):
            self.print_state()
        elif c == "status":
            self.status()
        elif c == "to":
            self.cmd_to(a)
        elif c == "home":
            self.cmd_named("ready")
        elif c == "named" and len(a) == 1:
            self.cmd_named(a[0])
        elif c == "d":
            self.cmd_delta(a)
        elif c == "jog":
            self.cmd_jog(a)
        elif c == "v" and len(a) == 1:
            value = float(a[0])
            if not 0.0 < value <= 1.0:
                raise ValueError("v must be in (0, 1]")
            self.scale = value
            self.print_settings()
        elif c == "planner" and len(a) == 1 and a[0].lower() in PLANNERS:
            self.planner = a[0].lower()
            self.print_settings()
        elif c == "plan" and len(a) == 1 and a[0] in ("on", "off"):
            self.plan_only = a[0] == "on"
            self.print_settings()
        elif c == "frame" and len(a) == 1 and a[0] in ("tcp", "base"):
            self.frame = a[0]
            self.print_settings()
        elif c == "step" and len(a) == 1:
            self.step = float(a[0])
            self.print_settings()
        elif c == "rstep" and len(a) == 1:
            self.rstep = float(a[0])
            self.print_settings()
        elif c == "set":
            self.print_settings()
        elif c == "grip":
            self.cmd_grip(a)
        elif c == "release":
            self.cmd_release(a)
        elif c == "gstate":
            self.print_gripper()
        elif c == "attach":
            self.cmd_attach(a)
        elif c == "detach":
            self.cmd_detach(a)
        elif c == "box":
            self.cmd_box(a)
        elif c == "history":
            print("\n".join(self.history) or "  (empty)")
        else:
            print("? unknown command, type `help`")
        return True


def main():
    # no rclpy SIGINT handler: Ctrl+C must cancel the goal, not shut down the node
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)  # Ctrl+C -> KeyboardInterrupt even if inherited as ignored
    shell = Shell()
    print("FR5 motion shell. Type `help`. Ctrl+C while a goal runs = cancel.")
    shell.status()
    try:
        while True:
            try:
                line = input(f"fr5[v{shell.scale:.2f} {shell.planner}{' plan' if shell.plan_only else ''} {shell.frame}]> ")
            except KeyboardInterrupt:
                print()
                continue
            except EOFError:
                break
            try:
                if not shell.handle(line):
                    break
            except (ValueError, KeyError) as e:
                print(f"? {e}")
    finally:
        shell.executor.shutdown(timeout_sec=1.0)
        shell.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
