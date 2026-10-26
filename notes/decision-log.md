# FR5 Hackathon: Decision Log (branch `feat/robot-motion-gripper`)

Committed in the branch under `notes/` since 2026-10-25 (R10; was local only, R7). Companion to the local `~/hackathon/CLAUDE.md`:
CLAUDE.md holds the **rules and current state**; this file holds the
**history: what was decided, which options were considered, why, and what
changed**. Append new entries; never delete old ones. If a value changes,
keep the old value and add the new one with the reason.

Entry format: ID, date, context, options, decision, why, status.
Status: FROZEN (decided), PENDING (waiting for the user), SUPERSEDED (replaced).

Index
1. Setup / install (S)
2. Repo / git workflow (R)
3. Environment / pixi (E)
4. Robot facts / geometry (G)
5. Robot description design, step 9 (D)
6. Control design / algorithms (C)
7. Open items / TODOs (T)
8. Commit log

---------------------------------------------------------------------------

## 1. Setup / install (S)

### S1: System packages (2026-10-25, FROZEN)
- `apt update && upgrade`, `git`, `curl` installed by the user before this log.
- Machine: MSI GP66, Ubuntu 24.04.3 LTS, git 2.43 (supports `worktree add --orphan`, needs >= 2.42).

### S2: pixi install (2026-10-25, FROZEN)
- Options: latest vs. specific version; user runs it vs. Claude runs it.
- Decision: **latest**, Claude runs `curl -fsSL https://pixi.sh/install.sh | bash`.
- Result: pixi **0.83.0** in `~/.pixi/bin/`; installer appended
  `export PATH="/home/arnav/.pixi/bin:$PATH"` to `~/.bashrc`.
- Telemetry: installer sent one anonymous ping (version, OS, arch).
  Opt-out (`PIXI_NO_TELEMETRY=1`) offered; user chose **defaults** (no opt-out).
- No CLI restart needed: Claude calls `~/.pixi/bin/pixi` by full path; new
  user shells pick it up via `.bashrc`.

### S3: Git identity + GitHub auth (2026-10-25, FROZEN)
- Identity (global): `user.name=vortexcv`, `user.email=<personal email>`.
- Auth options considered:

  | | `gh` CLI (HTTPS) | SSH key |
  |---|---|---|
  | Install | `sudo apt install gh` | nothing |
  | Login | `gh auth login`, browser | keygen + paste key on GitHub |
  | Extras | PRs/issues from terminal | git only |
  | Secret stored | token (keyring / `~/.config/gh`) | private key in `~/.ssh` |

- Decision: **`gh`** (one browser login, PR/issue tooling).
- Result: gh 2.45.0 (Ubuntu apt), account `vortexcv`, protocol https,
  scopes `gist, read:org, repo, workflow`; git credential helper =
  `gh auth git-credential` for github.com.
- User ran the sudo/interactive parts themselves (`! sudo apt install -y gh`, `! gh auth login`).

---------------------------------------------------------------------------

## 2. Repo / git workflow (R)

### R1: Reference checkout (2026-10-25, FROZEN)
- Cloned `AtharvaBhorpe/fr5-board-stacking` to `~/hackathon/fr5-board-stacking`
  (on `main`, HEAD `9568e45`). Our access: **WRITE**, repo PUBLIC.
- Remote branches at clone time: `main`, `feat/perception-geometric-detector`,
  `feat/perception-hardware-checks`.
- Submodules of the reference checkout NOT initialised (not needed for reading).
- User rule: **never change anything or push to `main`, not even by mistake.**

### R2: Push protection for `main` (2026-10-25, FROZEN)
- Options:

  | | pre-push hook blocking `main` | disable push URL of `origin` |
  |---|---|---|
  | Effect | refuses pushes to `refs/heads/main` | refuses every push |
  | Worktrees | shared, our branch still pushes | shared, would block our branch too |

- Decision: **pre-push hook** at `~/hackathon/fr5-board-stacking/.git/hooks/pre-push`
  (untracked, shared by all worktrees). Tested with fake input: main -> exit 1, feat -> exit 0.
- Caveat: `git push --no-verify` bypasses it; never use that flag.

### R3: Branch + worktree (2026-10-25, FROZEN)
- `git worktree add --orphan -b feat/robot-motion-gripper ~/hackathon/wt/feat-robot-motion-gripper`
- Orphan = no shared history with `main` (cannot open a GitHub PR; later
  integration via `git merge --allow-unrelated-histories`, accepted earlier).
- Branch is local only until first push (not pushed yet).

### R4: frcobot_ros2 submodule pin (2026-10-25, FROZEN)
- Path `external/frcobot_ros2`, URL `https://github.com/FAIR-INNOVATION/frcobot_ros2.git`.
- Options:

  | Ref | Commit | has `fairino_hardware_v3_9_0_1` | proven on our robot |
  |---|---|---|---|
  | **`main`** (2026-09-23) = teammate's pin | `fcf0c7f` | yes | **yes** |
  | tag `V3.0.0_RobotV3.9.2` (name matches controller) | `b6de550` | **no** (generic `fairino_hardware`) | no |
  | tag `V3.0.0_RobotV3.9.9_v1.1` | `5bed0b0` | yes | no |
  | branch `V3.0` | `33546af` | yes | no |
  | `master` | `a2c1793` | no (old `fr_ros2` layout) | no |

- Decision: **`fcf0c7f`** (git describes it as tag `V3.0.0_RObotV4.0.0`).
  Why: the only ref with the needed driver that is proven on our FR5.
- `ignore = dirty`: affects only `git status` display; **no effect on build
  time or size** (user condition). Kept.
- Submodule clone ~50 MB.

### R5: .gitignore (2026-10-25, FROZEN)
- Content: `.pixi/`, `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`,
  `.vscode/`, `.idea/`, `*.swp`, `.DS_Store` (typo `.DOS_Store` fixed before commit).
- `pixi init` later appended `.pixi/*` + `!.pixi/config.toml`: redundant
  (our `.pixi/` ignores the whole dir, so the exception can never match). Removed.
- Verification (user request): `.pixi/` had 119,117 files, 0 tracked, 0
  addable; `git check-ignore` matches `.gitignore:2:.pixi/` for the dir, a deep
  file and `config.toml`; `git add -A -n` listed only `.gitattributes`, `pixi.lock`, `pixi.toml`.
- `.gitattributes` from `pixi init` kept (`pixi.lock` = generated, binary merge, no diff).

### R6: Allowed vendor packages (2026-10-25, FROZEN)
- Old rule: use only `fairino_description` + `fairino_hardware_v3_9_0_1`.
- New rule: **+ `fairino_msgs`**. Why: `fairino_hardware_v3_9_0_1/package.xml`
  has `<depend>fairino_msgs</depend>`, used in `command_server.cpp/.hpp`.
  It is in the same submodule: only an extra colcon path, no pixi dep.
  User rule: add things when required and easy; include early if it helps
  confirm teammates' builds and has little impact.

### R7: Where our notes live (2026-10-25, FROZEN)
- CLAUDE.md and all rules/decision files are **local only**, never committed
  (they serve our own development; nobody else benefits).
- Extra .md files go in `~/hackathon/notes/` (this file).

### R8: Commit hygiene (FROZEN, from CLAUDE.md safety rules)
- `git diff --staged` before every commit. `pixi.lock` is hidden from
  `git diff` by `-diff`, so scan it directly (2026-10-25: 0 secret hits,
  only `https://prefix.dev` URLs).
- Commits local until the user says push.

### R9: Public email -> GitHub noreply, first push (2026-10-25, FROZEN)
- Repo is PUBLIC; real email was in commit author/committer, `pixi.toml`
  `authors`, `package.xml` `<maintainer>`. GitHub profile hides the email.
- Options: A push as is; B noreply `69069296+vortexcv@users.noreply.github.com`.
  Decision: **B**, set **repo-local** (`git config --local user.email`;
  global stays `<personal email>`). Worktrees share repo config, so the
  reference checkout uses it too (we never commit there).
- The 4 commits were still local -> rewritten with `git filter-branch`
  (env-filter for author/committer, tree-filter for the 2 files). Verified:
  old email 0 times in history, only the 2 email lines changed, submodule pin
  unchanged. Backup tag + `refs/original` deleted after the push.
- Hash mapping old -> new: 94d60ae -> b87ecf4, 68d7924 -> fa8866b,
  5d1c4d8 -> 2d63b24, 2da0a05 -> 8aefffb.
- Pushed: `git push -u origin feat/robot-motion-gripper` -> new remote branch
  at `8aefffb`, upstream set. `main` unchanged at `9568e45`.
- Rule from now on: rewriting is only OK before a push. Never rewrite/force-push the pushed branch.

### R10: Notes folder into the branch (2026-10-25, FROZEN, supersedes R7 for notes/)
- User: move `notes/` into the branch folder and commit it (backup + version control).
- Moved `~/hackathon/notes` -> `~/hackathon/wt/feat-robot-motion-gripper/notes` (tracked);
  `~/hackathon/notes` is now a symlink to it, so all paths in CLAUDE.md keep working.
- Before publishing (public repo): personal email redacted to `<personal email>` (R9); scan found no
  tokens / passwords. CLAUDE.md itself stays local (not part of this decision).
- Rule: notes are public now: never write secrets or the personal email into them.

---------------------------------------------------------------------------

## 3. Environment / pixi (E)

### E1: Workspace manifest (2026-10-25, FROZEN)
- Created with `pixi init` + `pixi add` (rule: deps only via `pixi add`).
- Name: **`fr5-robot-motion-gripper`** (`pixi init` defaulted to the folder
  name; renamed). No `hk_`: that prefix is for ROS packages only, and the pixi workspace is not one.
- Channels: `prefix.dev/robostack-humble` then `prefix.dev/conda-forge`.
- Dependencies (17): `ros-humble-desktop`, `colcon-common-extensions`,
  `ros-humble-moveit-ros-move-group`, `-moveit-planners`, `-moveit-kinematics`,
  `-moveit-configs-utils`, `-moveit-simple-controller-manager`,
  `-moveit-ros-visualization`, `-ros2-control`, `-ros2-controllers`, `-xacro`,
  `-joint-state-publisher-gui`, `compilers`, `cmake<4`, `ninja`, `pkg-config`,
  `sysroot_linux-64>=2.34`.
- Differences vs. teammate's manifest:

  | Item | Teammate | Ours | Why |
  |---|---|---|---|
  | `moveit-ros-warehouse` | yes | no | plan/state DB not needed |
  | `moveit-setup-assistant` | yes | no | SRDF/configs written by hand (F3); heavy GUI |
  | `[tasks]`, `[activation]` | yes | deferred | added when we first build |

- User rule: keep it **as light as possible**; add later only if needed.

### E2: glibc requirement (2026-10-25, FROZEN; first version SUPERSEDED)
- Why needed: some RoboStack packages need glibc newer than conda's default 2.17.
- v1 (SUPERSEDED): `[system-requirements] libc = "2.34"`. pixi 0.83 warns
  "deprecated in favor of virtual packages on `platforms`" on every run.
- v2 (current): `platforms = [{ name = "linux-64-glibc234", platform = "linux-64", glibc = "2.34" }]`.
  Re-lock result: **identical package set**, warning gone.

### E3: Install result + accepted warnings (2026-10-25, FROZEN)
- `.pixi/` = 5.7 GB; `pixi.lock` ~548 KB.
- Python resolved to **3.14.9** (conda-forge). Not pinned; `rclpy`, `tf2_ros`,
  `moveit_configs_utils` import fine. Revisit only if something breaks.
- Skipped post-link scripts (gdk-pixbuf, graphviz, gtk3 caches): accepted;
  enable with `pixi config set --local run-post-link-scripts insecure` only if RViz/GTK UI looks broken.
- CEP-42 channel-order warnings: harmless (our explicit order wins). Accepted.

### E4: Step 8 verification (2026-10-25, FROZEN)

| Need | Package (version) | Plugin |
|---|---|---|
| Pilz PTP/LIN | pilz_industrial_motion_planner 2.5.10 | `pilz_industrial_motion_planner/CommandPlanner` (+ PTP/LIN/CIRC loaders) |
| OMPL RRTConnect | moveit_planners_ompl 2.5.10 | `ompl_interface/OMPLPlanner` |
| IK | moveit_kinematics 2.5.10 | `kdl_kinematics_plugin/KDLKinematicsPlugin` (+ LMA, cached KDL) |
| Execution | moveit_simple_controller_manager 2.5.10 | ok |
| Arm controller | joint_trajectory_controller 2.54.1 | `JointTrajectoryController` |
| `/joint_states` | joint_state_broadcaster 2.54.1 | `JointStateBroadcaster` |
| Fake hardware | ros2_control | `mock_components/GenericSystem` |
| Controller mgr | ros2_control | `ros2_control_node`, `spawner` |

### E5: Build approach (2026-10-25, FROZEN pending task details)
- Question: teammate "uses pixi build"; is that different from colcon?
- Answer: teammate's `pixi.toml` has a **task** named `build` whose command
  is `CXXFLAGS="$CXXFLAGS -include unistd.h" colcon build --symlink-install --paths ...`.
  `pixi run build` = colcon inside the pixi env. Not the pixi-build backend.
- Ours: same approach (colcon via a pixi task). Exact task added at first build.

### E6: pixi `build` task + `-include unistd.h` flag (2026-10-25, FROZEN)
- Task: `CXXFLAGS="$CXXFLAGS -include unistd.h" colcon build --symlink-install --paths external/frcobot_ros2/fairino_description src/hk_fr5_description`.
  Run with `pixi run build`. Paths grow as packages are added (driver, `fairino_msgs`, our packages).
- Question (user): include the flag now, though no C++ is built yet?
  Decision: **yes**. It only affects C++ compiles (pre-includes a system
  header), so no effect on current packages, and it cannot be forgotten when
  the Fairino driver is added (that build fails without it).

### E7: Build error "no make" -> Ninja generator (2026-10-25, FROZEN)
- Error on first `pixi run build` (fairino_description):
  `CMake was unable to find a build program corresponding to "Unix Makefiles". CMAKE_MAKE_PROGRAM is not set.`
  followed by `CMAKE_C_COMPILER not set` / `CMAKE_CXX_COMPILER not set`.
- Cause: **`make` is installed neither in the pixi env nor on the system**.
  The compiler errors were a follow-on error (gcc/cc ARE in the pixi env).
  Teammate's identical manifest worked, most likely because their machine has
  system `make` (apt `build-essential`).
- Options:

  | | A: `pixi add make` | B: `[activation.env] CMAKE_GENERATOR = "Ninja"` |
  |---|---|---|
  | Change | new tiny dep (~0.5 MB) | one env line, no download |
  | Uses | default "Unix Makefiles" | `ninja` (already a dep) |
  | Speed | normal | slightly faster, parallel by default |
  | Teammate machines | works without system make | works (ninja from pixi) |

- Decision: **B** (no new dep, uses what is installed).
- Fix procedure: deleted `build/ install/ log/` first (CMake refuses to switch
  generator in an existing build dir), then rebuilt: 2 packages finished.
- Gotcha: `pixi workspace activation env add` needs `KEY=VALUE` (`CMAKE_GENERATOR=Ninja`), not two args.

### E8: Activation script + CMake min version (2026-10-25, FROZEN)
- `[activation] scripts = ["install/setup.sh"]`: `pixi run` / `pixi shell`
  source our colcon overlay, so `ros2` finds our packages. Added only
  **after** the first successful build (file must exist).
  If `install/` is deleted, run `pixi run build` again.
- Build printed `CMake Deprecation Warning: Compatibility with CMake < 3.10 will be removed`.
  Ours: `cmake_minimum_required` 3.8 -> **3.10** (warning gone). Vendor
  `fairino_description` still warns: harmless, we never edit `external/`.

### E9: Removed `ros-humble-joint-state-publisher-gui` (2026-10-25, FROZEN)
- Why: replaced by our `joint_sliders.py` (D-k); user rule "as light as possible".
- `pixi remove ros-humble-joint-state-publisher-gui` -> lock lost 3 packages,
  added none: `ros-humble-joint-state-publisher-gui`, `ros2-joint-state-publisher-gui`,
  `ros2-joint-state-publisher` (plain jsp, also unused: /joint_states comes
  from our sliders now and from ros2_control `joint_state_broadcaster` in steps 10-11).
- Rebuild OK; `ros2 pkg executables hk_fr5_description` -> `joint_sliders.py`;
  `joint_state_publisher_gui` -> "Package not found". User checks the viewer on their machine.

---------------------------------------------------------------------------

## 4. Robot facts / geometry (G)

### G1: Robot + controller (from CLAUDE.md, FROZEN)
- Fairino FR5 V6.0, controller v3.9.2, IP 192.168.58.2 (IP goes in config yaml, never in code).
- Driver `fairino_hardware_v3_9_0_1` (plugin `fairino_hardware/FairinoHardwareInterface`)
  moved the real robot in the teammate's setup despite controller v3.9.2.
- Fairino `ros2_cmd_server` conflicts with the motion driver; gripper I/O via
  XML-RPC `http://<ip>:20003/RPC2` works alongside.

### G2: Vendor URDF `fairino_description/urdf/fairino5_v6.urdf` (2026-10-25, verified)
- Robot name `fairino5_v6_robot`; root `base_link`; no ros2_control block.

  | Joint | Parent -> child | Origin xyz (m) | rpy | Limits (rad) | vel (rad/s) |
  |---|---|---|---|---|---|
  | j1 | base_link -> shoulder_link | 0 0 0 | 0 0 0 | ±3.0543 | 3.15 |
  | j2 | shoulder -> upperarm | 0 0 0.152 | 1.5708 0 0 | -4.6251 .. 1.4835 | 3.15 |
  | j3 | upperarm -> forearm | -0.425 0 0 | 0 | ±2.8274 | 3.15 |
  | j4 | forearm -> wrist1 | -0.39501 0 0 | 0 | -4.6251 .. 1.4835 | 3.2 |
  | j5 | wrist1 -> wrist2 | 0 0 0.1021 | 1.5708 0 0 | ±3.0543 | 3.2 |
  | j6 | wrist2 -> wrist3 | 0 0 0.102 | -1.5708 0 0 | ±3.0543 | 3.2 |

- Quirk: line 354, `wrist2_link` collision uses `<origins` (typo). Parser
  ignores it -> default zero origin, which is the intended value. Harmless; we never edit `external/`.

### G3: TCP distance (2026-10-25, CORRECTED)
- OLD (CLAUDE.md §1, wrong): "flange face -> relaxed cup tip 103.4 mm".
- NEW: gripper **mount face -> cup tip 103.4 mm**; mount plate 9.5 mm;
  **flange face -> cup tip 112.9 mm**.
- Evidence: `schmalz_gripper.stl` bbox z = 0 .. 0.1034 with origin at its
  mount face; teammate's xacro puts it 9.5 mm above the flange; teammate's
  handoff doc: "tcp = relaxed cup tip, 112.9 mm from the flange".
- Still TODO: ruler check on the real tool (adjust if off by > 1-2 mm).

### G4: Flange offset (2026-10-25, verified)
- Flange face = `wrist3_link` + 0.100 m along z (FR5 DH d6).
- Evidence: `wrist3_link.STL` bbox ends at z = 0.099.

### G5: Tool meshes (teammate's, allowed exception; 2026-10-25, verified)

| Mesh | Units | Origin | Bounding box (m) | Size |
|---|---|---|---|---|
| `schmalz_gripper.stl` | m | its mount face, +z to cup | x,y ±0.0315; z 0 .. 0.1034 | 1.08 MB |
| `d405_wrist_mount.stl` | m | flange face, +x along camera arm | x -0.037 .. 0.099; y ±0.0375; z 0 .. 0.0146 | 0.08 MB |

- D405 body 42 x 42 x 23 mm (approximate box; teammate value). Camera arm tilted 8 deg.

### G6: Frames vs. pendant (from teammate, FROZEN)
- Fairino `base_link` = pendant base rotated 180 deg about z:
  `x = -x_pendant`, `y = -y_pendant`, `z = z_pendant`. Pendant shows mm.
- TCP pointing down with yaw: quaternion (x, y, z, w) = (cos(yaw/2), sin(yaw/2), 0, 0)
  = Rz(yaw) * Rx(pi), matches our F7.

---------------------------------------------------------------------------

## 5. Robot description design, step 9 (D)

Package `hk_fr5_description` (ament_cmake). Robot name **`hk_fr5`** (FROZEN 2026-10-25).

TF tree added below the vendor chain:

    wrist3_link -(z 0.100)-> flange
      flange -(yaw = mount_yaw)-> d405_mount_link -> d405_link (body) -> d405_camera_link (x = viewing dir)
      flange -(z 0.0095)-> gripper_link -(z 0.1034)-> tcp   (tcp z = out of cup)

| # | Topic | Options | Decision | Why |
|---|---|---|---|---|
| D-a | Geometry numbers | A: `config/tool_geometry.yaml` via `xacro.load_yaml`; B: xacro properties | **A** | "everything in config/ yaml" rule |
| D-b | ros2_control block | A: added in `hk_fr5_moveit_config` (step 10); B: in description | **A** | MoveIt convention; description = pure geometry |
| D-c | Cup collision length | A: stop 5 mm short of tcp; B: reach tip (teammate) | **A** | final LIN onto board/table must not be "in collision" |
| D-d | `tool_adapter` frame | A: drop, gripper_link at flange + 9.5 mm; B: keep | **A** | fewer frames, same geometry |
| D-e | D405 mount yaw | yaml parameter, default 0.0 | **A** | real value still TODO (T3) |
| D-f | Camera frame names | A: `d405_link`, `d405_camera_link`; B: teammate's `wrist_camera_body/link` | **A** | |
| D-g | Build | colcon via pixi task (see E5), paths `external/frcobot_ros2/fairino_description` + `src/hk_fr5_description` | FROZEN | same as teammate's approach |
| D-h | Model-check launch file name | `view_robot.launch.py` / `check_urdf.launch.py` / `display.launch.py` | **`view_robot.launch.py`** (2026-10-25) | says exactly what it does; `display` is vague |

- Collision shapes (F13, sizes checked vs. meshes): mount ring cylinder,
  camera-arm box, D405 box 42x42x23 mm, gripper body cylinder dia 64 mm, cup cylinder dia 36 mm.
- Build/validation (2026-10-25): `xacro` OK; `check_urdf` OK, robot `hk_fr5`,
  tree base_link .. wrist3_link -> flange -> {d405_mount_link -> d405_link ->
  d405_camera_link, gripper_link -> tcp}. Computed: flange -> tcp = 0.0095 +
  0.1034 = 0.1129 m; cup collision z 0.0734 .. 0.0984 in gripper_link (5 mm short of tcp).
- D-h note (2026-10-25): user thought this file would be the package's main
  launch (motion + gripper). It is not: it only shows the URDF in RViz with
  joint sliders (robot_state_publisher + joint_state_publisher_gui + rviz2).
  The main bringup launch (move_group, controllers, motion_controller,
  gripper_controller) is step 11, in `hk_fr5_moveit_config`. A clearer name
  for the step 9 file was requested.

### D-l: Step 9 RViz check result (2026-10-25, PASSED)
- User ran `view_robot.launch.py` with `joint_sliders.py`; checklist items 1-8
  all OK (model loads, frames, tool stack, tcp at cup tip z out of cup,
  camera frame x = viewing dir, collision shapes/sizes match meshes, sliders
  move joints within limits, Zero all). No screenshots taken.
- tf2_echo (all joints 0):

  | Transform | Translation (m) | Rotation |
  |---|---|---|
  | flange -> tcp | [0.000, 0.000, 0.113] | identity |
  | flange -> d405_camera_link | [0.085, 0.000, 0.037] | quat [0, -0.755, 0, 0.656] = -98 deg about y (RPY [180, -82, 180]) |
  | base_link -> tcp | [-0.820, -0.315, 0.050] | RPY [90, 0, 0] deg (quat [0.707, 0, 0, 0.707]) |

  base_link -> tcp at zero matches FR5 dims: x = -(0.425 + 0.395),
  y = -(0.102 + 0.100 + 0.113), z = 0.152 - 0.102; tool points along -y.
  Use this to compare against the pendant at the same joint angles (mind G6: pendant x, y negated, mm).
- tf2_echo "Invalid frame ID ... frame does not exist" on start: harmless
  (asked before first TF message arrived).
- User observation: links can pass through each other with the sliders.
  Expected: robot_state_publisher / sliders / RViz do no collision checking.
  MoveIt (steps 10-11) checks self + scene collisions with these shapes;
  SRDF disables always-touching pairs. Caveat: motions NOT via MoveIt (direct
  controller trajectories, pendant jog) are not checked by MoveIt.

### D-i: joint_state_publisher_gui freezes (PyQt6) -> alias wrapper (2026-10-25, SUPERSEDED by D-k)
- Symptom (first `view_robot.launch.py` run): slider window freezes, desktop
  offers "force quit". Terminal:
  `AttributeError: 'QApplication' object has no attribute 'exec_'. Did you mean: 'exec'?`
  at `joint_state_publisher_gui.py` line 290 `sys.exit(app.exec_())`.
- Cause: RoboStack Humble builds the Qt Python stack on **PyQt6 6.11**
  (`ros-humble-desktop` -> rqt -> `qt_gui`/`qt_gui_cpp` -> `pyqt6`;
  `python_qt_binding` reports `pyqt 6.11.0`). jsp_gui **2.4.0** (only build
  available) still calls PyQt5's `exec_()`; PyQt6 renamed it `exec()`. Window
  is built, event loop never starts -> freeze. RoboStack packaging bug, not ours.
  Swapping to PyQt5 is not practical (whole desktop stack is built on PyQt6).
- Options:

  | | A: wrapper in our package | B: sed-patch installed file in `.pixi/` | C: no GUI (plain jsp, zeros) |
  |---|---|---|---|
  | Survives `pixi install` / teammates | yes | **no** | yes |
  | Sliders | yes | yes | no (check via MoveIt marker in step 11) |

- User asked: use PyQt6 directly instead of PyQt5 terms? Answer: we already
  run on PyQt6; only the vendor's `main()` uses the PyQt5 name. Sub-options:

  | | A1: alias wrapper (4 lines) | A2: own `main()` with `app.exec()` (~15 lines) | A3: own slider GUI (~150+) |
  |---|---|---|---|
  | PyQt5 names | one alias `exec_ = exec` | none | none |
  | Coupling to vendor internals | public `main()` only | GUI classes + `main()` wiring | none |
  | After upstream fix | no-op (`hasattr` guard) | our copy may drift | - |

- Decision: **A1**. `src/hk_fr5_description/scripts/jsp_gui.py`, installed via
  `install(PROGRAMS ...)` to `lib/hk_fr5_description`; `view_robot.launch.py`
  runs `hk_fr5_description/jsp_gui.py` instead of the vendor executable;
  `<exec_depend>python_qt_binding</exec_depend>` added.
- Test (offscreen, isolated ROS domain): GUI received robot_description, event
  loop ran, no `exec_` error, clean exit on SIGINT. `/joint_states` data flow
  could NOT be confirmed from Claude's sandboxed shell (DDS discovery there is
  flaky: even a plain test topic was listed but not echoable); user verifies in RViz.
- Testing gotcha: `pkill -f <pattern>` inside a Bash call can kill the calling
  shell itself (pattern is in its own command line) -> exit 144. Kill by PID.

### D-k: Own PyQt6 slider app `joint_sliders.py` (2026-10-25, FROZEN)
- 2nd error (user run, with A1 alias wrapper):
  `AttributeError: type object 'QFont' has no attribute 'Bold'` (jsp_gui line 84).
  Scan of `joint_state_publisher_gui.py` found 4 PyQt5-only names:
  `QFont.Bold` (84), `Qt.AlignRight` (90), `Qt.Horizontal` (98), `exec_()` (290).
- Options then: A = add 3 more aliases (~7 lines); B = own small PyQt6 app (~70 lines,
  can drop the jsp_gui pixi dep). User: **A for now; on any further error go straight to B.**
- 3rd error (Claude's offscreen test of extended wrapper, URDF passed as file arg):
  `AttributeError: type object 'QStyle' has no attribute 'PM_LayoutHorizontalSpacing'`
  in a second vendor file `flow_layout.py` (not covered by the first scan) -> process aborted.
  -> **Switched to B** per the user's rule. Alias wrapper `jsp_gui.py` deleted.
- Design (`src/hk_fr5_description/scripts/joint_sliders.py`, plain PyQt6 imports):
  - subscribes `/robot_description` (transient-local QoS), parses URDF with
    `xml.etree`, one slider per revolute/prismatic/continuous joint (continuous
    or no `<limit>` -> +-pi), URDF order.
  - publishes `/joint_states` at 10 Hz; "Zero all" button; label shows rad + deg.
  - single thread: `rclpy.spin_once(timeout 0)` from a 10 ms QTimer (no extra thread).
  - Ctrl+C in terminal closes it (SIGINT -> SIG_DFL).
  - Installed via `install(PROGRAMS ...)`; launch runs `hk_fr5_description/joint_sliders.py`.
  - package.xml: jsp_gui / python_qt_binding deps replaced by `rclpy`, `sensor_msgs`, `std_msgs`.
- Bugs found in own tests and fixed:
  1. v1 mapped 1000 steps over the range -> "zero" was -0.001 rad on j2/j4
     (asymmetric range). Fix: slider counts in **milliradians** (`SCALE = 1000`), 0 exact, 1 mrad resolution.
  2. v2 used `round()` for the range -> j2 max published 1.484 > limit 1.4835.
     Fix: range rounded **inward** (`ceil(lo*1000)`, `floor(hi*1000)`): never past a limit.
- Test (offscreen, URDF fed directly; no DDS discovery needed): 6 joints with
  correct limits, all 0.000 at start, 9 msgs/s on `/joint_states`, j2 max -> 1.483.
- ~~Leftover: pixi dep `ros-humble-joint-state-publisher-gui` now unused~~ removed in `8aefffb` (E9).

### D-j: KDL root-inertia warning (2026-10-25, ACCEPTED, revisit if needed)
- robot_state_publisher: `The root link base_link has an inertia specified in
  the URDF, but KDL does not support a root link with an inertia.`
- Cause: vendor `fairino5_v6.urdf` gives `base_link` an `<inertial>`.
- Effect: harmless (KDL ignores root mass; no dynamics used).
- Fix if ever needed: massless dummy root (e.g. `world`) fixed to
  `base_link`; `base_link` stays the planning frame (F1).
- Decision: **leave it for now** (user).

---------------------------------------------------------------------------

## 5b. MoveIt config design, step 10 (M)  (decided 2026-10-25)

Package `hk_fr5_moveit_config`. Findings behind these choices: `implementation-ref.md` sec. 2-8.

| # | Topic | Options | Decision | Why |
|---|---|---|---|---|
| M-1 | SRDF disable_collisions | A: hand-written (vendor 11 arm pairs + 6 rigid/touching tool pairs; wrist1/wrist2 vs tool stay enabled); B: temporary MoveIt Setup Assistant (heavy GUI) | **A** | rigid-pair rule is exact; no extra dep |
| M-2 | Controller name | `arm_controller` / vendor `fairino5_controller` | **`arm_controller`** | ours, matches group `arm` |
| M-3 | controller_manager `update_rate` (**SUPERSEDED by P-9: 50 Hz**) | 100 Hz (vendor, teammate proven) / 125 Hz (= driver ServoJ cmdT 0.008 s) | **125 Hz default + launch arg `update_rate` to switch to 100** | match driver period; keep a 1-line fallback if the real robot misbehaves |
| M-4 | Joint acceleration limit | keep vendor 0.7 rad/s^2 / raise | **0.7** | safe; slow (at 0.2 scaling ~4.5 s to reach speed); tune at step 19 |
| M-5 | Default scaling in joint_limits.yaml | 0.2/0.2 (F14) / vendor 0.1 | **0.2 / 0.2** | consistent with F14 |
| M-6 | IK solver | KDL / pick_ik 1.1.2 (review table in chat 2026-10-25, facts in implementation-ref sec. 7) | **KDL**, timeout 0.05 s | lowest error risk (ships with MoveIt 2.5.10), good LIN continuity; pick_ik only if KDL fails in practice |
| M-7 | Pipelines | only `ompl` + `pilz_industrial_motion_planner`; RRTConnect explicit default for `arm` | **yes** | no CHOMP; explicit default planner |
| M-8 | Named poses | `zero` (all 0), `ready` [0,-90,90,-90,-90,0] deg (tcp down at (-0.50,-0.10,0.36)); mock starts at `ready` | **yes** | FK-verified, above table side |
| M-9 | robot_description to ros2_control_node | topic remap / parameter (deprecated) | **topic remap** (step 11) | removes deprecation, future-proof |
| M-10 | Fairino driver build | now / step 19 | **both: throwaway compile now (separate build/install dirs), revert submodule, real build in step 19** | proves it compiles without leaving side effects |
| M-11 | Trajectory execution tolerances | MoveIt defaults / custom | **defaults** (start tol 0.01 rad, duration scaling 1.2, margin 0.5 s) | revisit on real robot |

- M-3 detail: `ros2_controllers.yaml` sets `update_rate: 125`; step 11 launch passes
  `{"update_rate": LaunchConfiguration("update_rate")}` after the yaml (later
  parameter wins). Verify with `ros2 param get /controller_manager update_rate`.
  **If the real robot stutters/jerks or ServoJ errors appear: try `update_rate:=100`
  (teammate's proven value) first.**
- Step 10 result (2026-10-25): package written, built, tested (xacro fake/real,
  SRDF vs URDF, MoveItConfigsBuilder load, 25 s runtime with mock hardware:
  125 Hz, controllers active, OMPL + Pilz loaded, move_group ready). Details:
  implementation-ref sec. 10b. Not committed yet.
- M-12 (2026-10-25, FROZEN): JTC `allow_nonzero_velocity_at_trajectory_end: false` (explicit).
  Why: future default, silences the deprecation warning; MoveIt trajectories end at zero velocity.
- M-13 (2026-10-25, FROZEN): **leave** the cosmetic "Cannot infer SRDF" warning (no fake `.setup_assistant`).
- M-14 (2026-10-25, FROZEN): SRDF state **`ready` = our HOME pose** (user). Name stays `ready`.
  [0, -90, 90, -90, -90, 0] deg, tcp down at (-0.497, -0.102, 0.364) m.
- M-15 (2026-10-25, fact, user question): start pose on the REAL robot = the robot's actual
  joints, not `initial_positions.yaml`. Driver `on_activate` reads `GetActualJointPosDegree(0)`
  (blocking) and copies it into the command buffer (holds still); `read()` reports actual joints.
  `initial_value` is used by the mock only. Nothing moves to `ready`/home automatically:
  MoveIt always plans from the current (actual) state.
- M-10 result (2026-10-25): throwaway compile OK (23 s, separate scratch dirs).
  Submodule typechange of 2 libfairino files reverted -> submodule clean, our
  build/install untouched, scratch deleted. Details + warnings: implementation-ref sec. 3.

---------------------------------------------------------------------------

## 5c. Bringup launch design, step 11 (L)  (decided 2026-10-25, all as recommended)

| # | Topic | Options | Decision |
|---|---|---|---|
| L-1 | Launch file name | `bringup.launch.py` / `robot.launch.py` / `moveit.launch.py` | **`bringup.launch.py`** |
| L-2 | Where the top-level launch lives | A: `hk_fr5_moveit_config` (CLAUDE.md package table), later steps add obstacle_loader / motion_controller / gripper_controller with on/off args; B: new `hk_fr5_bringup` | **A** |
| L-3 | `use_fake_hardware:=false` before the driver is built | refuse with clear error if package `fairino_hardware_v3_9_0_1` not found (ament index lookup) / let it fail in ros2_control | **refuse with clear error** |
| L-4 | RViz config | own minimal `moveit.rviz` (base_link, grid, MotionPlanning group arm, Loop Animation **off**) / copy vendor (loop on) | **own minimal** |
| L-5 | pixi shortcut | `pixi run sim` now, `robot` at step 19 / none | **`sim` now** |
| L-6 | Testing | Claude headless (`use_rviz:=false`, isolated domain: controllers, /joint_states, tf tcp, throwaway OMPL + Pilz LIN `/move_action` goals) + user RViz check / user only | **both** |

Launch args: `use_fake_hardware` (true), `update_rate` (125, int Hz, M-3), `use_rviz` (true).
- L-7 (2026-10-25, FROZEN): **C = A now, B later.** Pilz LIN failed with vendor Cartesian limits
  (Pilz checks joint motion vs UNSCALED joint acc 0.7; details implementation-ref 10c).
  Options: A lower Cartesian acc/dec; B raise joint acc (real FR5 value unknown, guessing is risky);
  C both. Done now: `pilz_cartesian_limits.yaml` acc 2.25 -> **0.3**, dec -5.0 -> **-0.3** m/s^2.
  Re-test (repo config): LIN at scaling 0.2: 5 cm and 15 cm SUCCESS (exact, 0.00 mm); LIN at
  scaling 1.0: FAIL (j4 0.83, j1 0.80 > 0.7; depends on the arm configuration OMPL left).
  => LIN reliable at default speed; fast LIN needs T9.
- Fact (2026-10-25): home pose tcp yaw = +pi/2 (quat [0.707, 0.707, 0, 0]).
- Step 11 user RViz check (2026-10-25): all 8 PASSED (home pose, MotionPlanning panel, OMPL
  Plan & Execute, Pilz LIN straight line at 0.2, PTP, goal state `ready`, collision goal red / no
  motion, clean Ctrl+C). Harmless messages seen: `Maybe failed to update robot state, time diff:
  0.004-0.031s` (RViz waits for a joint state newer than the trajectory end; mock timing), on shutdown
  `class_loader ... SEVERE WARNING ... unload library while objects ... exist` (known MoveIt exit noise),
  rviz `rcutils_set_error_state ... context is invalid` (publish after shutdown), interactive marker
  "Server not available while running". All processes "finished cleanly".

---------------------------------------------------------------------------

## 5d. Interface revision, step 12 (I)  (2026-10-25, user approved R1-R5)

The interfaces were frozen before implementation began. User asked for a fresh review against
steps 9-11. Changes (all **added fields**, frozen fields unchanged):

| # | Change | Why |
|---|---|---|
| I-1 (R1) | MoveTo `string named_target` ("" = pose; else SRDF joint state, "ready" = home, "zero") | pose goals let OMPL pick any arm configuration, which decided whether a later Pilz LIN worked (step 11); home must give the same joints every time |
| I-2 (R2) | MoveTo + MoveDelta result `moveit_msgs/MoveItErrorCodes error_code` | machine-readable failure reason for FMEA re-commanding (PLANNING_FAILED, NO_IK_SOLUTION, GOAL_IN_COLLISION, CONTROL_FAILED, PREEMPTED, INVALID_GOAL_CONSTRAINTS) |
| I-3 (R3) | MoveTo + MoveDelta `bool plan_only` | reachability checks without motion; plan-then-confirm on the real robot |
| I-4 (R4) | GetState `bool success`, `string message` | no valid state yet (no /joint_states or TF) must not look like real zeros |
| I-5 (R5) | Comments: RPY convention R = Rz(yaw) Ry(pitch) Rx(roll) (fixed axes), units, defaults; GetState yaw feeds MoveTo MODE_DOWN_YAW (home yaw = +pi/2) | avoid convention mix-ups between teams |

Not added: tolerances (config yaml, step 16), separate acceleration scaling (F14), frame_id (F1),
6-joint goal (named_target covers it). New dependency: `moveit_msgs` (already in the pixi env).

- Question (user): does home tcp yaw +90 deg need fixing? Answer: no. It follows from j6 = 0 and
  the tool mounting. Picking: cup is round, yaw irrelevant. Placing: board yaw = tool yaw, given
  explicitly per move_to. j6 +-175 deg keeps every yaw reachable. Revisit with T3 (D405 mount yaw):
  camera direction depends on tool yaw; changing home = one number (j6 of `ready`). -> T10.

---------------------------------------------------------------------------

## 5e. Static obstacles, step 13 (O)  (decided 2026-10-25, all as recommended)

| # | Topic | Decision | Why |
|---|---|---|---|
| O-1 | Extra box `floor_guard` (3 x 3 m slab just below table-top level) | **add** | table covers only x -0.92..+0.08; past the near edge OMPL could swing the arm below table height (real floor, cables, people) |
| O-2 | How boxes reach MoveIt | **`/apply_planning_scene` service** | one confirmed call, clear error; topic is fire-and-forget |
| O-3 | Node behaviour | **one-shot**: wait for move_group (timeout param), add all, log, exit; re-run to reload (same name = replace) | simplest |
| O-4 | Boxes removed from yaml | **document only** (restart bringup) | never remove objects others added (e.g. attached board, F12) |
| O-5 | yaml validation | **strict** (unique names, 3 positive sizes, 3-value position, numeric yaw; else error, nothing added) | |
| O-6 | Launch | **`use_obstacles:=true`** arg in bringup.launch.py | (2026-10-26: obstacle_loader merged into scene_manager, 5n T20d) |
| O-7 | z values | **absolute centres** in yaml (frozen format), top_z maths in comments | keeps frozen format |

Values = teammate estimates converted (implementation-ref 10e), all TODO-measure (T4).
- Step 13 result (2026-10-25): written, built, tested headless (validation, scene contents, state
  validity incl. floor_guard proof, plan-only goals, reload replace). Details implementation-ref 10f.
- Step 13 user RViz check (2026-10-25): all PASSED (4 boxes visible/listed, base on profile,
  into-table goal red + "Unable to sample any valid states for goal tree", above-table goal OK,
  around-the-post plan). One around-the-post Plan & Execute was refused once:
  "Computed path is not valid. Invalid states at index locations: [ 31 ] out of 184 ... contact
  between 'camera_post' and 'd405_link' ... Motion plan was found but it seems to be invalid
  (possibly due to postprocessing). Not executing." Retry with a slightly moved goal worked.
  Cause: OMPL collision-checks only at discrete steps; its path grazed the post corner with the
  D405 box; time parameterisation resampled the path and one new waypoint touched the post; MoveIt
  re-validated the final trajectory and refused (safety works). RRTConnect is random -> retry
  usually finds another path. -> T11 (retry in motion_controller, step 16; padding after T4).
  Other harmless RViz messages: "/recognize_objects not available" (unused panel feature),
  class_loader "namespace collision ... InteractiveMarkerDisplay" (known MoveIt RViz plugin linking
  warning), "Catastrophic failure" = MoveIt wording for an impossible goal.

---------------------------------------------------------------------------

## 5f. motion_controller skeleton, step 14 (C)  (decided 2026-10-25, all as recommended)

| # | Topic | Decision | Why |
|---|---|---|---|
| C-1 | Concurrency | **single-threaded executor + `async` callbacks** awaiting `/move_action` | rclpy 3.3.22 supports async callbacks; node stays responsive (get_state, cancel) without locks |
| C-2 | New goal while one runs (move_to and move_delta share one robot) | **preempt**: abort old with PREEMPTED, cancel its MoveIt goal, start new | F4 re-commanding |
| C-3 | Invalid input | **accept, then abort with INVALID_GOAL_CONSTRAINTS + clear message** | a REJECT gives the caller no reason |
| C-4 | Error mapping | invalid fields/NaN/scaling/mode/frame/planner/unknown named_target -> INVALID_GOAL_CONSTRAINTS (-16); `/move_action` missing -> COMMUNICATION_FAILURE (-25); no joint state/TF -> ROBOT_STATE_STALE (-23); preempted/cancelled -> PREEMPTED (-7); MoveIt failures passed through; skeleton valid goal -> FAILURE (99999) "not implemented yet" | |
| C-5 | Names | node `motion_controller`, namespace `motion` (launch can change it), relative `move_to` / `move_delta` / `get_state`; MoveIt names absolute | CLAUDE.md namespace rule |
| C-6 | Launch | `use_motion_controller:=true` in bringup.launch.py | |
| C-7 | Config | ROS parameter yaml `config/motion.yaml` | `ros2 param get`, launch override |
- Step 14 result (2026-10-25): skeleton written, built, tested (no-params error, names, params, 14 validation goals, get_state stub). Details implementation-ref 10h.
- C-8 (2026-10-25, user): planner names **case-sensitive** ("PTP", "LIN" only; "ptp" rejected).
- C-9 (2026-10-25, user): all-zero move_delta -> **succeed at once without moving** (step 17).

---------------------------------------------------------------------------

## 5g. get_state, step 15 (S)  (decided 2026-10-25, all as recommended)

| # | Topic | Decision |
|---|---|---|
| S-1 | TCP pose source | TF lookup base_link -> tcp, latest transform |
| S-2 | Joint source | subscription to /joint_states (latest msg + arrival time) |
| S-3 | success=false when | no joint state yet / latest older than state_timeout (1 s) / TF lookup fails; specific message each |
| S-4 | Joint order | always the planning chain order j1..j6 (derived from /robot_description, not hard-coded) |
| S-5 | RPY | R = Rz(yaw) Ry(pitch) Rx(roll); tool down -> roll = +-pi, pitch 0, yaw feeds MODE_DOWN_YAW; gimbal lock (pitch = +-90 deg) -> yaw = 0, roll takes the rotation |
| S-6 | tcp_pose stamp | the TF transform's own timestamp |
| S-7 | TF listener | node's own listener on the single-threaded executor (no extra thread, C-1) |

- **User note (2026-10-25): the RPY extraction convention may need to change; check it on the real
  robot (orientation may differ from sim) -> T12.**
- Step 15 result (2026-10-25): written, built, tested (offline RPY + chain tests, runtime home/after-move, 3 failure messages, clean SIGINT). Two bugs found and fixed (ros2_control <joint> in chain parsing; ExternalShutdownException on Ctrl+C). Details implementation-ref 10i.

---------------------------------------------------------------------------

## 5h. move_to, step 16 (T)  (decided 2026-10-25)

| # | Topic | Decision |
|---|---|---|
| T-1 | Goal tolerances | **relaxed (user): position 1 mm (sphere radius), orientation 0.01 rad (~0.57 deg) per axis** (was provisional 0.5 mm / 0.005 rad) |
| T-2 | named_target joint tolerance | **relaxed (user): 0.01 rad** per joint (proposal was 0.001) |
| T-3 | Retry | OMPL only, on PLANNING_FAILED (-1) / INVALID_MOTION_PLAN (-2), up to `replan_retries` (2), each retry logged; Pilz never (deterministic) |
| T-4 | Phase feedback | map MoveIt feedback (PLANNING -> PLANNING, execution/MONITOR -> EXECUTING); one /move_action call (F5) |
| T-5 | Pilz with named_target | allowed (PTP = joint move; LIN = straight tcp line to that pose) |
| T-6 | Messages | "<CODE_NAME>: <hint>" with short hints for common codes |
| T-7 | EXECUTING phase (T-4 mapping impossible: MoveIt Humble /move_action feedback is only PLANNING / IDLE) | options: A watch arm_controller FollowJointTrajectory action status; B plan + /execute_trajectory; C drop EXECUTING -> **A** (user) |
| T-8 | named_target joint tolerance (0.01 rad gave 4-6 mm tcp error on OMPL `ready`) | (a) keep 0.01; (b) 0.001 for named targets only; (c) auto Pilz PTP (no obstacle avoidance) -> **(b) 0.001 rad** (user, T-2 superseded) |
- T-9 (user, 2026-10-25): "if relaxed tolerances produce too much position difference, tighten them
  everywhere (named and pose goals)" -> measure pose goals with tight tolerances (0.1 mm / 0.001 rad)
  vs relaxed (1 mm / 0.01 rad) and decide on data.
- T-10 (2026-10-25, applied under the user's T-9 rule): **tighten everywhere**: position 0.0001 m,
  orientation 0.001 rad, joint 0.0001 rad (T-1, T-2, T-8 superseded). Data (same goals, headless):
  relaxed pose 1 mm / 0.01 rad -> errors up to 1.00 mm and 0.57 deg (right at the limit); tight
  0.1 mm / 0.001 rad -> <= 0.1 mm, <= 0.05 deg; planning time (goal -> EXECUTING) 0.03-0.06 s in
  both, 0 retries. Named `ready` via OMPL: joint tol 0.01 -> 4-6 mm, 0.001 -> 0.4-0.9 mm,
  0.0001 -> 0.03-0.08 mm (3 runs), same planning time. Pilz exact in all cases.
- Step 16 user check (2026-10-25): all 6 CLI checks PASSED (OMPL, LIN straight, plan_only no motion,
  into-table GOAL_STATE_INVALID, ready at 0.5 within 0.01 mm, get_state). Speed question raised ->
  measured (implementation-ref "Speed analysis"): planning 12-69 ms, execution limits dominate. -> T-11 (pending).
  User then noted the delay was mostly between entering the CLI command and the first motion ->
  confirmed: CLI 0.5-2.2 s before the goal is even accepted; persistent client 12-14 ms to motion start.
- T-11 (2026-10-25, user: "do all 1-3"): (1) joint max_acceleration 0.7 -> **2.0 rad/s^2** (all joints),
  Pilz Cartesian acc/dec 0.3/-0.3 -> **1.0/-1.0 m/s^2** (same ratio as L-7); (2) higher speed = task code
  sets velocity_scaling per goal (default stays 0.2: F14 + real-robot <= 20 % first-run rule; no code
  change); (3) planning_time 5.0 -> **1.0 s** (plans take 12-70 ms; failures now reported in 1 s).
  Evidence: config "B" of the speed test = these values (sequence 13.8 -> 8.1 s at 0.2, 5.3 s at 0.5,
  all Pilz PTP/LIN succeeded). Full re-test skipped (user: only limits changed). Untested: planning_time 1.0
  on hard OMPL plans (retry T-3 covers it). Real FR5 limits still to replace 2.0 (T9).
  Open: blended motion (Pilz sequence, no stops inside a pick) -> T13.

---------------------------------------------------------------------------

## 5i. move_delta, step 17 (U)  (2026-10-25; user: "review the proposal on your side and decide yourself")

| # | Topic | Decision (Claude, reviewed) | Why |
|---|---|---|---|
| U-1 | Current pose | `read_state()` (TF base_link -> tcp); stale -> ROBOT_STATE_STALE | same source as get_state |
| U-2 | Target maths | 3x3 rotation matrices: FRAME_TCP p' = p + R d, R' = R Rd; FRAME_BASE p' = p + d, R' = Rd R; Rd = Rz(dyaw) Ry(dpitch) Rx(droll) | frozen semantics (CLAUDE.md 5), same RPY convention as get_state |
| U-3 | All-zero delta | SUCCESS at once, no MoveIt call (C-9) | user |
| U-4 | Planner | always Pilz LIN (F6), tight pose tolerances (T-10), no retry (deterministic), plan_only supported | |
| U-5 | Code | shared preparation helper for move_to + move_delta (MoveIt idle, state, server) | no duplicate logic |
| U-6 | Offline tests | hand-derived tool-down expectations, apply/invert round trips, independent quaternion implementation in the test | |
| U-7 | Runtime tests | each delta's end pose vs expected (TCP/base translations, pure rotations, combined, zero, plan_only, into table) | |

- Sign note (correction of a wrong statement in chat): tool down -> tcp z-axis = -z of base_link, so
  FRAME_TCP dz = +0.10 moves DOWN (toward the board), dz = -0.10 moves UP; FRAME_BASE dz = +0.10 moves UP.
- Step 17 result (2026-10-25): written, built, tested (offline maths vs independent implementation, 11 runtime cases <= 0.001 mm, move_to regression). Details implementation-ref 10k. User CLI check passed (6/6).

---------------------------------------------------------------------------

## 5j. Cancel / preempt + smooth stop, step 18 (V)  (2026-10-25; user: fix what breaks, decide yourself;
##     added requirement: "on the real robot the stop is fast but smooth, no high jerks / joint forces")

Bug found (test 1): **MoveIt Humble /move_action accepts a cancel but does NOT stop execution**
(log: "cancel requested" at .36, "Goal reached, success!" at .91; E1 result SUCCESS). So cancel /
preempt never stopped the robot.

Experiment (mock = exact commanded motion; metrics from the controller's commanded reference):
| Method | stop | peak acc after | peak jerk after |
|---|---|---|---|
| normal motion | - | ~1.0 rad/s^2 | ~50-60 rad/s^3 |
| E1 cancel MoveIt goal | never (motion finishes) | - | - |
| E2 cancel controller goal (JTC holds position) | 9 ms | 30.9 | 2578 |
| E3 1-point decel trajectory (closed loop) | 501 ms | 30.6 | 1965 (JTC restarts from v = 0) |
| **E3 2-point decel trajectory (q, v) -> (q + vT/2, 0)** | **509 ms** | **1.00** | **83** |
| E3 1-point with open_loop_control true | 512 ms | 1.39 | 25 |

| # | Decision | Why |
|---|---|---|
| V-1 | on cancel / preempt during execution: send a 2-point deceleration trajectory to the arm controller (start = latest commanded position + velocity, end = q + v T / 2 with v = 0, T = max_j(|v_j|) / stop_deceleration, min 0.05 s); MoveIt then aborts its execution (CONTROL_FAILED), our goal ends PREEMPTED | only working stop; smooth (constant deceleration, linear velocity ramp from cubic Hermite); no controller config change |
| V-2 | wait for the stop trajectory to finish before ending the goal | next goal must not plan from a moving robot (allowed_start_tolerance) |
| V-3 | cancel during planning: if the stopped goal's trajectory starts anyway (MoveIt ignores cancel), stop it at once | MoveIt executes after planning even if cancelled |
| V-4 | `stop_deceleration: 2.0` rad/s^2 (= joint acc limit): a stop is never harsher than a planned motion; tied to T9 | user: fast but smooth |
| V-5 | controller_state at 125 Hz (was 50) + extrapolate by its age | fresh start state for the stop trajectory |
| V-6 | known trade-off: the robot coasts v^2/(2a) along its current direction (not collision-checked; continues the collision-free planned direction, a few cm at our speeds) instead of freezing | smoothness requirement |
- Step 18 result (2026-10-25): bug fixed (smooth stop via controller), 19/19 headless checks incl.
  full-speed cancel (peak acc 2.01 rad/s^2 vs 31 for a hard stop). Details implementation-ref 10l.
- V-7 (2026-10-25, user: "keep it simple, stop harder at high speeds"): coast problem (V-6, T14) solved by a
  speed-dependent deceleration instead of a collision check:
  a = clamp(v_max^2 / (2 * stop_max_coast), stop_deceleration, stop_deceleration_max)
  with stop_deceleration 2.0 rad/s^2 (= joint acc limit, used at normal speeds), stop_max_coast 0.1 rad
  (~6 deg on the fastest joint), stop_deceleration_max 5.0 rad/s^2 (2.5x the planner limit, still ~6x
  below the hard stop's 31 rad/s^2). Effect: |v| <= 0.63 rad/s (default 0.2 scaling) -> unchanged
  (2.0, coast <= 0.1 rad); 1.18 rad/s -> a 6.96 capped 5.0 -> coast 0.14 rad (was 0.35); full 3.15 rad/s ->
  5.0 -> coast ~1.0 rad (only above ~2.2 rad/s does the coast exceed 0.5 rad).
  Options considered: (a) collision-check the stop end state (more code, a check delays the stop);
  (b) stop harder at high speed (chosen, simple); (c) hard stop above a speed (jerk spike).
  **Revisit when:** (1) real FR5 joint acceleration limits are known (T9: stop_deceleration and
  stop_deceleration_max scale with them); (2) at the first real-robot cancel test in step 19 (<= 20 %
  speed stays in the 2.0 regime: check the stop feels smooth and the controller reports no errors);
  (3) before task code uses velocity_scaling > ~0.5 (where the cap matters). T14 closed by V-7.
  V-7 test (2026-10-25): suite 19/19; full speed 1.20 rad/s -> decel 5.00, stop 240 ms, coast 0.144 rad (was 0.35), peak acc 5.01, jerk 376 (hard stop ~31 / ~2600); normal-speed stops unchanged (2.0).

---------------------------------------------------------------------------

## 5k. Pre-real-robot preparation + new step order (P)  (2026-10-25, user)

Open items reviewed after step 18 (A = doable now without robot, B = needs user/robot, C = later).

| # | Decision (user) |
|---|---|
| P-1 | **A4 (code-enforced speed cap for first real runs) DROPPED**: keep it simple, trust each caller (safety rule 2 stays a human rule). |
| P-2 | Do now, in order: A1 real driver build (+ LD_LIBRARY_PATH; **accept** the submodule link change Q3-5, hidden by ignore=dirty), A2 `pixi run robot` task, A3 real-mode dry run without robot, A5 real-robot run plan, A6 housekeeping (close T11). |
| P-3 | B2 (T9 real joint limits): **online search now**; use the found values as **vendor-ref values** (marked as such in config + log) so they can be replaced if the web app reports different ones. |
| P-4 | B3 (realtime scheduling for ros2_control): explained; not done now; in the run plan as a fix to try if the real robot stutters (with update_rate:=100). |
| P-5 | **New step order**: real-robot test moves to the END. Next: pre-real-robot prep (P-2/P-3) -> gripper (T1 questions first) -> small Qt test app (discuss after the gripper, like joint_sliders) -> real-robot test (<= 20 %, e-stop, explicit confirmation). |
- P-6 (2026-10-25): vendor-ref values (online): joint speed 180 deg/s = 3.1416 rad/s (catalogue via qviro/welectron),
  TCP 1 m/s, acceleration 360 deg/s^2 = 6.283 rad/s^2 ("full performance" web setting, fairino.be distributor
  page; Fairino publishes no official acc limit), J3 +-160 deg (URDF +-162). Test with acc 6.283 + Pilz 2.25:
  sequence 13.8 -> 5.2 s @0.2, but LIN @1.0 violated j4 decel (6.49 > 6.283) and cancel suite 15/19.
- P-7 (2026-10-25, lab closes in 1 h): **today uses the tested config** (acc 2.0, Pilz 1.0, 19/19); only the
  J3 +-160 deg limit applied (pure restriction). Vendor-ref acc to be re-tested next session.
  Done: A1 driver build + LD_LIBRARY_PATH (libfairino.so.2 resolves; submodule link change accepted), A2
  `pixi run robot`, A6. Postponed (no time): A3 dry run, A5 written plan, gripper (step 20), Qt app (step 21).
  Later status (2026-10-26): A3 superseded by the first real session (P-9), A5 covered by notes/test.md,
  gripper DONE (step 20a-e), Qt app replaced by motion_shell (P-8).
- P-8 (2026-10-25, user): instead of the Qt test app, a **command shell** (`motion_shell`, `pixi run shell`):
  faster to build and debug; all features of the proposed app mapped to commands (state, status, to / named /
  home, d, jog with step/frame, v / planner / plan settings, live phases, result + code + message + final pose,
  history, Ctrl+C = cancel). Units mm / deg. Tested in sim (all commands, error cases, Ctrl+C cancel ->
  PREEMPTED + smooth stop). Test gotchas: background processes in a non-interactive bash inherit SIGINT as
  IGNORED (shell now restores `signal.default_int_handler`); grep patterns in cleanup must not match the own
  command line (R-5). Manual CLI steps: notes/test.md. Qt app (step 21) no longer needed unless asked.
- **P-9 (2026-10-25, real FR5, user: "50 Hz is the best setting") controller_manager update_rate 125 -> 50 Hz.**
  Symptom: every command started 1-2 s late on the real robot and a cancel stopped the arm only after it had
  swept most of the path (sim: instant). Debugged before changing values (user's call):
  `notes/lag_probe.py` (controller_state: commanded `reference` vs read-back `feedback`):
  | update_rate | loop measured | lag start / end | max commanded-actual |
  |---|---|---|---|
  | 125 Hz (M-3) | 79 Hz | +2.07 / +2.12 s | 0.052 rad (2.96 deg) |
  | **50 Hz** | 50 Hz | **+0.10 / +0.06 s** | **0.0078 rad (0.44 deg)** |
  Cause: the FR5 controller queues the ServoJ points the driver streams every cycle (cmdT 0.008 s hard-coded
  in the vendor driver); at 79-125 points/s it falls behind and the queue settles ~2.1 s deep -> start and
  stop both late. At 50 points/s no queue builds. The smooth stop (5j) and the limits were not the cause.
  Applied: ros2_controllers.yaml update_rate 50, state_publish_rate 50; bringup default `update_rate:=50`.
  Pitfall during the test: `update-rate:=50` (hyphen) is silently ignored by ros2 launch (log still said
  125 Hz) -> always check the `controller_manager update_rate: N Hz` log line. (~~Todo: reject unknown launch args.~~ DONE T22.)
  ~~Open: confirm motion is smooth at 50 Hz~~ CLOSED 2026-10-26 (user: very smooth, 5m).
- Real-robot session result (2026-10-25, user: "ok this works now"): at 50 Hz moves start at once and
  cancel / smooth stop works on the real FR5. Next session: step 20 gripper (T1 questions first).
  Still open: ~~smoothness check at 50 Hz~~ (closed, 5m), reject unknown launch args (X-7), vendor-ref acceleration re-test (P-6/P-7),
  T3 / T4 / T5 / T6 / T12 measurements.

---------------------------------------------------------------------------

## 5l. Gripper controller, step 20 (W)  (decided 2026-10-25, user)

T1 answers (user, 2026-10-25):
- Schmalz gripper, 24 V DC, own vacuum pump and own blow-off pump (no compressed air).
- Tool **DO0** = vacuum pump (1 on, 0 off); tool **DO1** = blow-off pump (1 on, 0 off);
  tool **DI0** = vacuum achieved (1 = vacuum reached = part gripped, 0 = no vacuum).
- Sequencing (when to grip / release) belongs to the control loop, not to this sub-project.
- Grip: DO1 off, DO0 on, start timer (3 or 5 s, configurable); DI0 = 1 within the timeout -> keep DO0 on,
  report success; else DO0 off, report failure. Release: DO0 off, DO1 on for 1 s, DO1 off.
- I/O access: XML-RPC `http://<robot_ip>:20003/RPC2` (Python stdlib xmlrpc.client), `SetToolDO(id, status,
  smooth, block)`, **`GetToolDI(id, block)`** (tool inputs; `GetDI` = control-box inputs, user + SDK header
  `robot.h` agree), `GetToolDO`. All three exist as XML-RPC method names in libfairino.so.2.3.0; the
  XML-RPC return format of GetToolDI / GetToolDO is **unverified** (check read-only at the lab, 20f).
  [Superseded 2026-10-27, 5o: GetToolDI = `[0, x]`; GetToolDO is not an XML-RPC method and crashed the controller (H-2 / H-4).]

| # | Topic | Decision |
|---|---|---|
| W-1 | Command interface | actions `/gripper/grip` (Grip.action), `/gripper/release` (Release.action) + topic `/gripper/state` (GripperState.msg, latched, 10 Hz + on change); options were B services, C topics (no cancel, no result link) |
| W-2 | Grip timeout | yaml `grip_timeout` 3.0 s + per-goal override (0 = default); same for `blow_off_time` 1.0 s |
| W-3 | Cancel during GRIPPING | DO0 off, result CANCELLED |
| W-4 | DI0 drops while HOLDING | report only (state PART_LOST + message), pump stays on, control loop decides |
| W-5 | Grip while HOLDING with DI0 = 1 | immediate SUCCESS, no I/O switching |
| W-6 | Release with nothing held | always the full sequence (harmless) |
| W-7 | Release during GRIPPING | aborts the grip (PREEMPTED), then releases |
| W-8 | DI0 debounce | 2 equal reads in a row (poll 50 ms -> 100 ms) |
| W-9 | Node start | ~~read + publish the I/O state, switch nothing (safety rule 3)~~ **replaced by H-7** (2026-10-27): both pumps off at start |
| W-10 | Node shutdown | **DO0 and DO1 both off** (user; proposal was DO1 off only) |
| W-11 | F12 attach board box to tcp | later, in the control loop (knows the board size) |
| W-12 | Distinct error states (user: no shared ERROR tag) | state: IDLE, GRIPPING, HOLDING, RELEASING, PART_LOST, COMM_ERROR (XML-RPC unreachable / timeout), CONTROLLER_ERROR (controller error code); result error_code: SUCCESS, NO_VACUUM (timeout, pump off), CANCELLED, PREEMPTED, COMM_FAILURE, CONTROLLER_ERROR, INVALID_GOAL. A new grip / release clears the error states (no reset command). No PART_LOST result code (user: the grip timeout covers vacuum lost before success) |
| W-13 | COMM_ERROR recovery (user, 2026-10-26; amends W-12) | **COMM_ERROR clears by itself** on the first good read: ~~outputs re-read (controller may have restarted)~~ (no DO read-back since H-2), back to the state before the error; HOLDING without vacuum -> PART_LOST; unreachable at start -> the start reset (H-7) runs when the robot appears. Debounced DI is reset during the outage (first read after it counts at once). PART_LOST / CONTROLLER_ERROR still cleared only by a new grip / release |
| W-14 | Goal concurrency (20c, Claude; user confirmed 2026-10-26) | any new goal preempts the running one (grip / release, both actions; same rule as motion_controller): the old goal ends PREEMPTED and switches nothing, the new goal takes over the outputs. Extends W-7 to grip-during-grip, grip-during-release, release-during-release |
| W-15 | Cancel during release (20c, Claude; user confirmed 2026-10-26) | blow-off off at once, state IDLE, CANCELLED (vacuum pump is already off) |
| W-16 | I/O failure inside a goal (20c, Claude; user confirmed 2026-10-26) | try both pumps off, result COMM_FAILURE / CONTROLLER_ERROR (message says which DOs are confirmed off). DI unreadable for the whole grip -> COMM_FAILURE at the timeout (not NO_VACUUM). Grip start resets the debounced DI to False: success needs `debounce_reads` fresh reads after pump on |
| W-17 | motion_shell without gripper (user, 2026-10-26: "use_gripper false -> shell must not crash or work improperly") | `grip` / `release` / `gstate` check the gripper node first (grip server, 1 s) and say "gripper node not running: use_gripper:=false?"; a missing node drops the cached state (no stale data); `status` shows the gripper line; result wait checks the server every 0.5 s -> "action server disappeared" instead of hanging if a node stops mid-goal (also for motion goals) |
- Fake mode: in-memory I/O, DI0 = 1 `fake_vacuum_delay` 0.5 s after DO0 on; runtime param `fake_vacuum`
  true/false to test the failure path. Single-threaded executor + async callbacks (like motion_controller),
  XML-RPC socket timeout 0.5 s. Config `hk_fr5_gripper/config/gripper.yaml` (IP, port, I/O numbers, times).
- Sub-steps: 20a interfaces, 20b package + fake I/O + state topic, 20c grip/release actions + tests,
  20d launch `use_gripper` + build task, 20e optional motion_shell commands, 20f lab checks (confirmation).

---------------------------------------------------------------------------

## 5m. Branch review (X)  (2026-10-26, user answers to the review of all open / closed items)

| # | Topic | Decision (user) |
|---|---|---|
| X-1 | Lost test scripts (steps 15-18 scratchpads gone) | OK **as long as every test is documented** (implementation-ref 10i-10l, 10n). D1: commit the surviving gripper test scripts + mock XML-RPC server with run commands. Motion tests are **not** rebuilt (user confirmed) |
| X-2 | Attached board (F12, W-11 amended) | **Only our nodes talk to MoveIt.** The control loop needs our API to attach a picked board (variable w, l, h) to `tcp`, and detach it. Design pending |
| X-3 | Dynamic obstacles (D3) | assume the control loop needs to add / remove **dynamic stack obstacles** (placed boards, stack). Same API family as X-2. Design pending |
| X-4 | Driver hides a lost connection (Q3-1) | OK for a demo; ~~plus the cheap check (frozen read-back warning)~~ dropped 2026-10-26 (user: existing implementation enough, T21 closed) |
| X-5 | CONTROLLER_ERROR not auto-cleared (W-12 vs W-13) | keep |
| X-6 | Gripper start with robot unreachable shows pumps off (unknown) | accept |
| X-7 | Unknown launch args silently ignored (P-9 pitfall) | reject them with a clear error listing the valid names |
| X-8 | Team README (API for teammates) | final goal of this session, after the other items |
| X-9 | T13 blended motion, T8 IK benchmark | parked (optional) |
| X-10 | Gripper XML-RPC polling (20 Hz) next to the motion driver | other teams run both at once; test tomorrow at the lab (lag_probe while the gripper polls, T17) |
- **P-9 smoothness CLOSED** (user, 2026-10-26: "the motion was very smooth with 50 Hz").
- T12: the teammate repo only verified the **position** mapping (x = -x_pendant, y = -y_pendant, mm; `tf2_echo base_link
  flange` vs web UI, tool 0); no orientation / RPY data. Our tcp has the flange orientation (pure z offsets). If the
  pendant uses the same fixed-axis X-Y-Z convention: R_base = Rz(pi) R_pendant -> **roll = rx, pitch = ry, yaw = rz +- 180 deg**
  expected. Verify at the lab (tool 0 on the pendant: position differs by the 112.9 mm tool length along tool z).
- T5 vs T6: T5 = arm motion check (a commanded delta gives the same real travel, e.g. dz 0.10 -> 100 mm, measured with a
  ruler / pendant; tests units, kinematics, driver). T6 = tool geometry check (real flange -> cup tip length vs our
  112.9 mm in the URDF; an error shifts every tcp target and makes rotations about the tcp wobble). T5 passes even
  with a wrong tool length for pure translations.

---------------------------------------------------------------------------

## 5n. Scene objects API, T20 (S)  (decided 2026-10-26, user: "all recommended", S-4 amended)

Problem found in the design: a board attached while resting on the table (or placed onto the stack) is in
contact -> MoveIt rejects the lift (START_STATE_IN_COLLISION) / the place goal. Fix: attached box shorter at its
bottom by a contact margin.

| # | Topic | Decision |
|---|---|---|
| S-1 | Node | new node `scene_manager` in `hk_fr5_motion`, **obstacle_loader merged into it** (one node owns the planning scene). Only our nodes talk to MoveIt (X-2) |
| S-2 | ROS names | `/scene/*` (MoveIt does not use `/scene`) |
| S-3 | Services | `/scene/attach_board` (length, width, height, yaw vs tcp x, dx / dy offset of the board centre from the cup; box under the cup attached to `tcp`, touching the gripper allowed, one board at a time); `/scene/detach_board` (keep_as_obstacle: true -> stays as a fixed obstacle where it is, the stack grows; false -> removed); `/scene/add_box` (name, size, position, yaw in base_link, obstacles.yaml format); `/scene/remove_box` (name, "" = all dynamic boxes; static obstacles.yaml boxes cannot be removed). Responses: success, message |
| S-4 | Contact margin | **default 2 mm (yaml) and also settable per request** (user) |
| S-5 | attach only while gripper HOLDING? | no: the caller's job (F12 rule documented); nodes stay independent |
| S-6 | Name of a kept detached board | `stack_<n>`, returned in the message |
| S-7 | motion_shell | `attach L W H`, `detach [keep]`, `box add / rm` commands |
- Merge safety (user: "be extra extra careful"): motion_controller has **no** obstacle code (checked 2026-10-26: only
  error hints + an empty `planning_scene_diff` with `is_diff = True` in every MoveGroup goal, which must stay: it means
  "current scene, no change"). obstacle_loader touches: bringup `use_obstacles` arg + Node, setup.py entry point, package
  descriptions, obstacles.yaml, notes (5e O-*, test.md). OMPL's FixStartStateCollision adapter may nudge start states
  that are slightly in collision: keep in mind when testing the margin.
- Sub-steps: T20a interfaces, T20b scene_manager (static boxes = obstacle_loader behaviour, then the services),
  T20c tests in sim, T20d launch (obstacle_loader removed), T20e motion_shell commands.
- **T20d done (2026-10-26):** bringup starts `scene_manager` (namespace scene, scene.yaml + `load_static: use_obstacles`)
  instead of obstacle_loader; scene_manager **always runs** (the control loop needs /scene/*), `use_obstacles` keeps its
  meaning (static boxes on / off). obstacle_loader.py, its entry point and its stale installed script removed; package
  descriptions updated. motion_controller untouched by T20d. Historical entries (step 13, 5e O-*) stay as history.

### 5n-1. T20a interfaces (user approved 2026-10-26) + collision model: metrics and reasons (check here first on collision issues)

Interfaces (`hk_fr5_interfaces/srv`): `AttachBoard` (length, width, height, yaw, dx, dy, contact_margin),
`DetachBoard` (keep_as_obstacle -> success, message, **name**), `AddBox` (name, size[3], position[3], yaw),
`RemoveBox` (name, "" = all dynamic). Two user-approved choices on top of S-3 / S-6:
| # | Choice | Why |
|---|---|---|
| S-8 | DetachBoard returns the kept name in its own field `name` (not only in `message`) | the control loop uses it (later remove_box / logging) without parsing text |
| S-9 | A kept board keeps **exactly its attached shape and pose**, incl. the margin at its bottom | its top = the real board top (that is what the next place lands on), and the next board gets its own margin -> no contact ever between stack layers |
| S-12 | Lift after attach (measured T20c, see "Why a contact margin") | control loop lifts with move_delta (Pilz LIN), never OMPL move_to from the contact pose | OMPL's start-state fix jumped 0.26-1.02 rad from the real state when the board touched the table |
| S-11 | Pick-zone board added as an obstacle (E-3) | API unchanged (user: no `replace_box` field); **rule: the control loop always calls remove_box for that box before attach_board** (documented in AttachBoard.srv) | an overlap gives START_STATE_IN_COLLISION on every motion |
| S-10 | contact_margin per request, 0 -> yaml default 0.002 (S-4) | same "0 = default" rule as Grip.timeout / Release.blow_off_time / velocity_scaling; a CLI call without the field gets the safe default. Side effect: an exact 0 margin is impossible (use 1e-6) |

**Attached box model** (tcp frame; tcp z points out of the cup = DOWN when the tool points down):
box centre = (dx, dy, (height - margin) / 2) in the **tcp frame**, box turned by `yaw` about tcp z (board x = tcp x at
yaw 0); (corrected 2026-10-26: a first draft said "in the board frame"; the srv comment was right);
box size = (length, width, height - margin) -> the box spans tcp z = 0 (cup tip = board top) .. height - margin.
Attached to link `tcp`; touch_links = gripper_link + tcp (the cup sits on the board).

**Clearances from the attached board top (tcp z = 0) to the tool collision shapes** (tool_geometry.yaml, computed
2026-10-26, pitch -8 deg of the camera arm included): cup cylinder (r 18 mm) **5.0 mm** (= cup_collision_gap),
gripper body cylinder (r 32 mm) 30.0 mm, D405 body box 73.0 mm (it sits at x 64.5..109.3 mm, i.e. above a 270 mm board),
camera arm box 98.3 mm. -> no tool shape touches the board; touch_links only cover the real cup contact.

**Why a contact margin, why 2 mm** (CORRECTED 2026-10-26 with T20c measurements; the first draft said "Pilz LIN
fails at once on a start state in collision" -> **wrong**):
- A box resting on a surface is at distance 0 and FCL counts that as a collision (measured: margin 1e-6 m ->
  `/check_state_validity` reports `board <-> table`, move_group logs "Found a contact between 'table' and 'board'").
- **Pilz LIN does not check the start state**: with margin 1e-6 m the lift was planned AND executed (SUCCESS, tcp
  -0.039 -> 0.061). So Pilz alone would not need the margin.
- **OMPL silently replaces a colliding start state** (request adapter "Fix Start State In Collision", jiggle_fraction
  0.05, max_sampling_attempts 100): measured from the contact state, attempt 1 "Found a valid state near the start
  state at distance **1.022** (joint space, rad) after 2 attempts" -> path invalid at index 0, 2 -> our retry (T-3)
  -> attempt 2 jumped **0.261** -> "SUCCESS". **The plan then starts 0.26-1.0 rad away from where the robot really is**
  (real robot: a jump at the first point, at best an abort on MoveIt's start tolerance). -> **This is the main reason
  for the margin**: after attach the robot state must be collision free so OMPL plans from the real state.
- Also: a collision-free state keeps `/check_state_validity` (our attach warning) meaningful: a warning then always
  means a real problem (E-2 / E-3).
- 2 mm: clearly above numeric noise, below the board thickness (teammate board 270 x 200 x 6 mm -> modelled
  **4 mm thick, 33 % shorter**). Same principle as the 1 mm gaps of obstacles.yaml (O-*), larger because a picked board's
  pose is less certain than a static box (cup compression, vision, table estimate). Nothing of the robot fits in 2 mm.
- Must be < height (validated); for thin boards (<= 2 mm) pass a smaller margin per request.
- **Rule for the control loop (S-12, follows from the measurement): lift after attach with move_delta (Pilz LIN),
  never with an OMPL move_to straight from the contact pose.** If the margin is not enough (table higher than the
  estimate, E-2), Pilz still lifts from the real state. **T25 done:** the OMPL "Fix Start State In Collision" adapter
  is removed -> OMPL now fails clearly with -26 START_STATE_INVALID instead of jumping (measured).

**Stack model:** detach(keep) -> `stack_<n>` with the attached shape (S-9): top = real top of the placed board, bottom
2 mm above the layer below (floating gap, harmless). Next place: tcp goes to stack top + height -> the new attached
box's bottom is 2 mm above stack_<n> top.

**Known model errors / assumptions (debug here first):**
| # | Item | Effect | Sign of trouble |
|---|---|---|---|
| E-1 | Cup compression under vacuum (unknown, est. a few mm): the real board top sits above tcp z = 0 | modelled board lower than the real one by the compression -> conservative toward the table; real board closer to the gripper body (30 mm gap, OK) | real board hits something MoveIt thinks is clear above the board: unlikely |
| E-2 | Table top ~~-0.045 estimate~~ **-0.060 measured** (2026-10-27, H-10) | board-on-table contact depends on it; margin covers <= 2 mm error | START_STATE_IN_COLLISION right after attach on the table -> check the table height (H-10) or pass a larger margin |
| E-3 | A pick-zone board added as a dynamic box (add_box) still exists at attach time | attached box overlaps it -> START_STATE_IN_COLLISION | remove_box(that name) **before** attach_board (control loop, rule S-11) |
| E-4 | Descent onto a board / the stack | cup collision ends 5 mm short of tcp -> tcp can reach a box top without collision | GOAL_STATE_INVALID at the pick height -> the box top is above the commanded tcp z |
| E-5 | MoveIt padding assumed 0 (world + robot links, defaults) | margins above are exact | to verify in T20c |
| E-6 | Place height: tcp target z = stack top (or table) + height | if the target is lower than that, the goal collides | GOAL_STATE_INVALID at place -> check the stack top z |
| E-7 | yaw / dx / dy wrong (vision) | box sticks out on one side | collisions only on one side of the board |

T20b implementation notes: kept board pose = TF base -> tcp composed with the attached box pose (S-9); stack index
monotonic (names never reused, also after remove_box ""); after attach / add_box the current state is checked with
`/check_state_validity` and contacts are appended to the message as a WARNING (success stays true) -> first place to
look for E-2 / E-3. Reentrant callback group (implementation-ref Q20-5: default group deadlocked).
Validation (T20b): length, width, height > 0 and finite; yaw, dx, dy finite; contact_margin >= 0, finite, < height;
add_box sizes > 0, finite values, non-empty name, not a static name, not "board".

## 5o. Lab 2026-10-27: tool I/O over XML-RPC (T15 / 20f, part of T16) (H)

Real controller (FR5 V6.0, v3.9.2, 192.168.58.2), plain `xmlrpc.client` from the laptop, no driver running.
Details + raw replies: implementation-ref 10o.

| # | Finding | Consequence |
|---|---|---|
| H-1 | `GetToolDI(id, block)` -> `[0, 0]`: Python `list` of 2 `int` = `[error, value]` (error 0 = OK, value 0/1). Same for `(0, 0)`, `(0, 1)`, `(1, 0)`: `block` 0 / 1 makes no visible difference for a read. DI0 = 0 with pump off, and stayed 0 for 1.5 s with pump on and no part (expected) | `parse_reply` format assumption confirmed, no change |
| H-2 | `GetToolDO()` -> `xmlrpc.client.Fault -506: "Method 'GetToolDO' not defined"`. The C++ SDK reads tool DOs from the realtime state packet (`tl_dgt_output_l`, bit0-1), not via XML-RPC | gripper_controller start / recover read-back (`gripper_io.get_do`) always fails on the real robot -> stuck in COMM_ERROR. **User 2026-10-27: remove GetToolDO everywhere; assume every SetToolDO is applied by the controller** (DO state = last commanded) |
| H-3 | `SetToolDO(id, status, 0, 0)` -> `0` (`int`, OK). DO0 = 1: vacuum pump ran (user confirmed); DO0 = 0 off. DO1 = 1 / 0: blow-off on / off (user confirmed: it blew off) | DO mapping (5l) confirmed for both pumps; SetToolDO call as implemented |
| H-4 | XML-RPC port 20003 refused connections for a few minutes (`ConnectionRefusedError`, ping OK), starting right after the `GetToolDO` fault (the next call, `system.listMethods`, was already refused); later calls worked again. **User: the GetToolDO call crashed the complete controller** | Never call GetToolDO (or any unknown RPC method) on the controller; node already treats a lost controller as COMM_ERROR + retry (W-13) |
| H-5 | Sequence test (user confirmed): DO0 on 5 s -> off, DO1 on 2 s -> off, DI0 read every 100 ms (83 reads, all `[0, x]`, read latency 1.5 / 1.9 / 4.4 ms min / avg / max). DI0: 0 until 1.1 s after pump on, then 1 until pump off, 0 within 100 ms of pump off, 0 during blow-off and after. User held a substrate board near the cup, the cup sucked it in | DI0 = vacuum achieved confirmed with a board. **1.1 s is not a property of the gripper**: time to vacuum varies greatly per pick (debug log value only, no tuning from it). grip_timeout 3 s stays. 50 ms polling is cheap (~2 ms per read) |
| H-6 | User rule: **vacuum pump and blow-off pump must never be on at the same time** | Checked in gripper_controller (see implementation-ref 10o): grip = DO1 off before DO0 on, release = DO0 off before DO1 on, a failed first switch stops the second, preempted goals switch nothing, blocking calls on a single-threaded executor (no interleaving) |
| H-7 | Node start (user 2026-10-27, replaces W-9): **switch vacuum pump and blow-off pump off, then IDLE** = known state, published; DO states afterwards = last commanded (no read-back, H-2). Start reset failed (unreachable / error code) -> retried every `comm_retry_period` (also for CONTROLLER_ERROR, which W-13 would not clear); **grip / release refused with COMM_FAILURE until the reset has run**, so a late reset can never cut a running goal (added by me: race found while implementing) | A board held when the node starts is dropped (accepted by the user: known state first). Recovery after COMM_ERROR keeps the last commanded DO states; a controller restart that resets its outputs is caught by DI0 (HOLDING -> PART_LOST) |
| H-8 | Interlock (user approved): inside `switch()`, switching a pump **on** first switches the other pump off; if that call fails, the exception stops before the pump is switched on. Callers no longer switch the other pump themselves | Rule H-6 holds by construction for all code paths. Same SetToolDO sequence on the wire as before (grip (1,0), (0,1); release (0,0), (1,1), (1,0)), no extra call |
| H-9 | motion_shell `gstate` right after shell start said "no /gripper/state received yet" (latched state not delivered yet). User: "you decide" -> **wait up to 1 s** for the first state, then the error ("... within 1 s") | First `gstate` in shell_gripper_check now shows IDLE; user: code ok, no full test run needed |
| H-10 | Table height (teammate measurement 2026-10-27, updated `fr5_bringup/config/workcell.yaml`): table top -> robot base = **40 mm aluminium profile + 20 mm spacer (within the profile) = 60 mm** -> table_top_z -0.045 -> **-0.060**. Camera post replaced by a tripod at about the same height, a few cm beside the table. Teammate's board variants / zones: not for us (user). Options for the spacer: A one box profile + spacer, B two boxes (needs spacer x/y) -> **user: A**, camera post z per formula | `obstacles.yaml` (header formulas): table z -0.060 -> -0.075 (top -0.060); profile 0.043 -> **0.058** high, z -0.0225 -> -0.030 (spans -0.059 .. -0.001, box rendered under the robot, no gap); camera_post size / x / y kept (user: OK for demo), z 0.806 -> 0.791; floor_guard z -0.056 -> -0.071 (arm limit -0.046 -> -0.061). Effect: picks from the table become plannable (with -0.045 a board top at -0.054 lay inside the modelled table); pick / place heights 15 mm lower; the old hidden 15 mm margin is gone. No code change (scene_manager reads the yaml). scene_motion_check TOP and README example -> -0.060 |

---------------------------------------------------------------------------

## 6. Control design / algorithms (C)

Frozen 2026-10-25, before this log (full text and interfaces in CLAUDE.md §5).
The rationale below is only what was recorded at the time.

| ID | Topic | Decision | Recorded why |
|---|---|---|---|
| F1 | Frame | everything in Fairino `base_link` | web app only as human reference |
| F2 | TCP | all targets for `tcp` (relaxed cup tip) | |
| F3 | MoveIt config | own `hk_fr5_moveit_config`, own SRDF, group `arm`, chain `base_link -> tcp` | Fairino SRDF ends at `wrist3_link`, not our TCP |
| F4 | Interface | ROS 2 actions (`hk_fr5_interfaces`) | cancel/preempt needed for FMEA re-commanding |
| F5 | Implementation | Python, raw rclpy clients to `/move_action`, TF for state | no C++, MoveItPy or pymoveit2 (simple) |
| F6 | Planners | `move_to`: OMPL RRTConnect default (obstacle avoidance), optional Pilz PTP/LIN; `move_delta`: always Pilz LIN | straight TCP line for deltas |
| F7 | Orientation | default TCP down + yaw, R = Rz(yaw) Rx(pi); FULL = rpy | boards picked from above |
| F8 | Deltas | dx..dyaw, TCP frame (default) or base frame | |
| F9 | Gripper | separate `gripper_controller` node in `hk_fr5_gripper` | details: 5l (W-*) |
| F10 | Joint state | `/joint_states` + `/motion/get_state` | |
| F11 | Obstacles | yaml + loader node, collision boxes in planning scene | not in URDF |
| F12 | Attached board | attach box to `tcp` after vacuum confirmed | |
| F13 | Tool model | visual = teammate STLs; collision = primitives | |
| F14 | Velocity | 0 -> 0.2, same for acceleration | first real runs <= 20 % |

- ROS names: `/motion/move_to`, `/motion/move_delta`, `/motion/get_state`,
  `/gripper/*`. No `hk_` in ROS names (it marks package ownership only).

---------------------------------------------------------------------------

## 7. Open items / TODOs (T)

Lab checklist (only the open items, in order, with checks + expected results): `notes/test.md` section 0.

| # | Item | Status |
|---|---|---|
| T1 | ~~Before step 20 ask: Schmalz model, tool DO suction, DO blow-off, DI vacuum-achieved, lift/blow-off control~~ | DONE: 5l (DO0 vacuum, DO1 blow-off, DI0 vacuum achieved) |
| T2 | ~~Pilz available in RoboStack Humble~~ | DONE (E4) |
| T3 | D405 mount yaw on the real robot | open |
| T4 | ~~Measure workcell (table, profile, camera post) via `/motion/get_state`~~ | CLOSED 2026-10-27 (user: doesn't really matter for the demo): table height measured -0.060 (H-10); table x/y + camera post kept as estimates |
| T5 | ~~Commanded vs. real distance (`move_delta dz=0.10` -> 100 mm)~~ | CLOSED 2026-10-26 (user): assumed correct for the demo |
| T6 | ~~TCP ruler check (112.9 mm flange -> cup tip)~~ | CLOSED 2026-10-26 (user): assumed correct for the demo |
| T7 | ~~D-h launch file name~~ | DONE: `view_robot.launch.py` |
| T9 | Get real FR5 joint acceleration (and velocity) limits from the Fairino web app / datasheet before step 19; then raise `joint_limits.yaml` max_acceleration (0.7 vendor default) and re-test Pilz LIN at high scaling (L-7 B). | open |
| T10 | Home pose yaw (+pi/2 at j6 = 0): revisit with T3 when camera use is designed; change = j6 of SRDF `ready` (+ initial_positions.yaml). | open |
| T11 | ~~Step 16: OMPL re-plan on INVALID_MOTION_PLAN / PLANNING_FAILED~~ | DONE in step 16 (T-3, `replan_retries` 2). Padding / longest_valid_segment_fraction not needed so far. |
| T12 | (expected result: 5m) Step 19: check GetState RPY on the real robot: same pose on pendant vs `/motion/get_state` (pendant base is rotated 180 deg about z, G6; Fairino may use another RPY convention). Change the extraction / interface comment if it does not match expectations. | open |
| T13 | Optional after step 18: blended / continuous motion (Pilz MoveGroupSequence: approach -> descend -> retreat without stops), e.g. a `move_sequence` action, if stop-and-go still matters. | open, optional |
| T14 | ~~collision-check stop end state / stop harder at high speed~~ | DONE: V-7 (speed-dependent deceleration); revisit points listed in V-7 |
| T15 | ~~20f-1 lab: XML-RPC reply format of GetToolDI / GetToolDO (read-only first) + meaning of `block`~~ | DONE 2026-10-27: 5o H-1..H-4. GetToolDI = `[err, value]` ints; **GetToolDO not defined, crashed the controller** -> removed everywhere (H-2, H-4); H-5 DI0 with a board, H-6 pumps never both on, H-7 start reset, H-8 interlock |
| T16 | 20f-2 lab: real grip / release with a board (confirmation); time_to_vacuum, debounce, vacuum decay after release | open (lab); raw SetToolDO / GetToolDI sequence with a board DONE (H-5): vacuum gone within 100 ms of pump off; left: `grip` / `release` actions via the node |
| T17 | 20f-3 lab: `notes/lag_probe.py` while the gripper polls; motion lag must stay ~0.1 s (X-10) | open (lab) |
| T18 | 20f-4 lab: first `pixi run robot` with the gripper node (real launch never run) | open (lab) |
| T19 | Vendor-ref acceleration 6.283 rad/s^2 re-test (P-6/P-7); then revisit V-7 stop values, L-7 Pilz limits, M-11 execution tolerances | open (lab) |
| T20 | ~~Scene objects API for the control loop~~ | DONE 2026-10-26: 5n (T20a interfaces `e84e6ba`, b scene_manager `5a18bf0`, c tests `9838d04`, T25 `c3f3843`, d launch `dd593ac`, e motion_shell) |
| T21 | ~~Frozen read-back warning in motion_controller (X-4)~~ | CLOSED 2026-10-26 (user): existing implementation is enough for the demo; not built |
| T22 | ~~Reject unknown bringup launch args (X-7)~~ | DONE 2026-10-26: `ARGS` table in bringup.launch.py feeds the declarations + `_check_unknown_args` (first thing in `_setup`): unknown name -> exit 1 before any process, error lists valid names + underscore hint |
| T23 | ~~Commit gripper test scripts + mock XML-RPC server (X-1 / D1)~~ | DONE 2026-10-26: `tests/` (11 scripts + mock_fairino.py, testenv.py, scene_lib.py, README.md); no hard-coded paths (ament index), logs in /tmp/hk_fr5_tests; all re-run from the repo: 27, 20, 27, 17, 28, 6 (+ T26 INFO) checks + printed scripts OK; scene_smoke rewritten with asserts (17) for the T20d bringup; obstacle_loader comparison dropped (its reference values live in scene_launch_check.py) |
| T25 | ~~Remove OMPL request adapter "Fix Start State In Collision"~~ | DONE 2026-10-26 (user: remove): OMPL from a colliding start now fails with -26 START_STATE_INVALID (T20c re-run 28/28, Pilz unaffected); motion_controller hint for -26 added |
| T26 | Ctrl+C of the bringup: several nodes end "process has died, exit code -2" instead of "finished cleanly" (pre-existing, same with the pre-T20d launch: SIGINT reaches the process group AND ros2 launch forwards a 2nd SIGINT). Risk on the real robot: ros2_control_node may die before the driver's on_deactivate (StopMotion / CloseRPC). Check at the lab (T18) whether the driver logs its stop. Python nodes (motion_controller, scene_manager) show a `KeyboardInterrupt` traceback inside `destroy_node` from the same 2nd SIGINT (harmless, no hardware); gripper_controller ignores signals during its switch-off (Q20-4) | open |
| T24 | ~~Team README: run, ROS names, interfaces, result codes (X-8)~~ | DONE 2026-10-26: `README.md` (quick start, bringup args, ROS API, conventions, result codes, pick / place sequence + rules S-11 / S-12, rclpy example, packages / config, known limitations); the rclpy example was extracted from the README and run against the sim bringup (pick -> grip -> attach -> lift -> place -> release -> detach keep -> stack_0, exit 0) |
| T8 | OPTIONAL IK benchmark KDL vs pick_ik (`/compute_ik`, same random tcp poses: success rate, time, joint distance to seed; throwaway `pixi add`). **Only if KDL fails noticeably** (IK failures in move_to/move_delta/RViz drag). Not in the step plan. | open, optional |

---------------------------------------------------------------------------

## 8. Commit log (branch `feat/robot-motion-gripper`; pushed up to `1f18c83` on 2026-10-27)

| Commit | Content |
|---|---|
| `b87ecf4` | submodule `external/frcobot_ros2` @ `fcf0c7f` + `.gitignore` |
| `fa8866b` | pixi workspace: `pixi.toml`, `pixi.lock`, `.gitattributes` |
| `2d63b24` | hk_fr5_description (xacro, tool_geometry.yaml, meshes, view_robot launch/rviz, joint_sliders.py) + pixi build task / Ninja / activation |
| `8aefffb` | drop unused `ros-humble-joint-state-publisher-gui` (E9) |
| `d3afc04` | hk_fr5_moveit_config (SRDF, ros2_control xacro, KDL, limits, OMPL + Pilz, controllers 125 Hz) + build task |
| `35cabe1` | bringup.launch.py + moveit.rviz + Pilz Cartesian acc/dec 0.3 + `pixi run sim` |
| `f67cf4f` | hk_fr5_interfaces (MoveTo, MoveDelta, GetState; R1-R5 additions) |
| `ff859c7` | hk_fr5_motion: obstacles.yaml + obstacle_loader, use_obstacles launch arg |
| `1e7fece` | motion_controller skeleton + motion.yaml + use_motion_controller launch arg |
| `5f9ce4b` | get_state service (joints, TF tcp pose, rpy) + 2 fixes |
| `2004505` | move_to (OMPL/Pilz, named targets, plan_only, EXECUTING via controller status, retries) + tight tolerances + faster limits |
| `cf5a745` | move_delta (Pilz LIN, TCP/base frame, zero delta, shared _prepare) |
| `76ef200` | cancel / preempt fix: smooth stop via arm controller, speed-dependent deceleration |
| `58e64da` | hk_fr5_interfaces: Grip, Release, GripperState, GripperErrorCode + decisions 5l |
| `7cf694c` | hk_fr5_gripper: gripper_controller (state topic, tool I/O, start/shutdown, W-13 recovery) |
| `569644e` | hk_fr5_gripper: grip / release actions (W-14..W-16) |
| `50d563d` | bringup use_gripper, build task, gripper SIGINT/SIGTERM handlers (Q20-4), test.md C1 |
| `14ad348` | motion_shell: grip / release / gstate, works without the gripper node (W-17) |
| `dd8140d` | notes: branch review 5m, open items T15-T24, housekeeping |
| `910674a` | notes: T20 scene objects design (5n), T5 / T6 closed for the demo |
| `e84e6ba` | hk_fr5_interfaces: AttachBoard, DetachBoard, AddBox, RemoveBox + collision model notes 5n-1 |
| `5a18bf0` | hk_fr5_motion: scene_manager (static boxes + /scene/* services, reentrant callback group Q20-5) |
| `9838d04` | T20c scene vs planning tests (27/27), margin rationale corrected with measurements |
| `c3f3843` | moveit_config: OMPL FixStartStateCollision adapter removed (T25) + -26 hint |
| `dd593ac` | bringup starts scene_manager, obstacle_loader removed (T20d), T26 noted |
| `f650c78` | motion_shell: attach / detach / box commands (T20e) |
| `5bcfa93` | bringup rejects unknown launch arguments (T22), T21 closed |
| `dab07e2` | tests/: gripper + scene checks, mock Fairino XML-RPC server (T23) |
| `3f7d178` | README.md: team API documentation (T24) |
| `bb90ed2` | notes: commit table up to `3f7d178` |
| `57e742c` | notes: lab checklist 2026-10-27 in test.md section 0 |
| `f984b66` | gripper: GetToolDO removed, start reset both pumps off, pump interlock (5o H-1..H-8, T15) |
| `1f18c83` | table top -0.060 measured (H-10, T4 closed), compact team README, shell gstate wait (H-9), scene tests follow the table |
