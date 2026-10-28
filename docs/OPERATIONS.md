# Operations guide — FR5 board stacking workcell

[Back to the visual README](../README.md)

**Use this guide for detailed setup, operation and troubleshooting.**
Run all commands from the repository root.

A Fairino FR5 robot arm with a Schmalz vacuum gripper moves boards on a table.
An operator gives jobs by voice or text in a web app.
A local language model (Gemma) turns free text into jobs.
The main controller does the jobs in priority order and recovers from errors.
The job pipeline runs on one Linux PC. Setup downloads software and model weights.

This guide tells you how to install, start, use, test and repair the system.
It uses the rules of ASD-STE100 Simplified Technical English.

## Contents

1. [System overview](#1-system-overview)
2. [Hardware and workcell](#2-hardware-and-workcell)
3. [Repository layout](#3-repository-layout)
4. [Install on a new workstation](#4-install-on-a-new-workstation)
5. [Start the simulation](#5-start-the-simulation)
6. [Start on the real robot](#6-start-on-the-real-robot)
7. [Use the web app](#7-use-the-web-app)
8. [Use the command line](#8-use-the-command-line)
9. [Stop the system](#9-stop-the-system)
10. [Logs](#10-logs)
11. [Tests](#11-tests)
12. [Configuration files](#12-configuration-files)
13. [ROS interfaces](#13-ros-interfaces)
14. [Troubleshooting](#14-troubleshooting)
15. [Known limits](#15-known-limits)
16. [Design records](#16-design-records)

## 1. System overview

The system has five parts. Each part is one or more processes.
The launcher, `tools/stack.py`, starts and monitors them from one terminal.

| Part | Folder | What it does |
|---|---|---|
| Motion and gripper | `src/hk_fr5_motion`, `src/hk_fr5_gripper`, `src/hk_fr5_moveit_config`, `src/hk_fr5_description`, `src/hk_fr5_interfaces` | Moves the tool tip with MoveIt 2. Switches the vacuum pump and the blow-off pump. Keeps the MoveIt planning scene up to date. |
| Vision 1 (overhead camera) | `vision1/` | Finds boards and zone sheets in the image of the overhead camera. Publishes their positions on the table in metres. |
| Vision 2 (wrist camera) | `vision1/wrist.launch.py`, `config/d405.yaml` | Starts the RealSense D405 on the wrist. The web app shows its colour image and its depth image. |
| Main controller | `src/hk_fr5_task`, `src/hk_fr5_task_interfaces`, `tools/` | Keeps a world model of boards and zones. Runs the job queue and the skills (pick, place, home, wait). |
| Conversation and web app | `conversation/`, `web/` | Web app for phones, tablets and the PC. Speech to text (Parakeet), text to speech (Phonon), the planner (Gemma). |

The data flows in this sequence:

1. The overhead camera sends images to Vision 1 (`vision1/node.py`).
2. Vision 1 publishes boards and zones as JSON on `/vision/boards`.
3. The world model (`world_model`) tracks each board and publishes `/task/world`.
4. The operator types or says a request in the web app (`web/gateway.py`).
5. The chat bridge (`conversation/bridge.py`) reads the request.
   - A fixed command (for example `stop` or `transfer VARa drop_off_a 2`) goes to the main controller without change.
   - A free request goes to the planner (`conversation/planner.py`). Gemma proposes commands. The planner repairs and checks each command against a fixed grammar.
6. The bridge sends each command to the main controller through the agent bridge (`tools/agent_bridge.py`).
7. The main controller (`task_main_controller`) checks each job again and queues it.
8. The skill server (`skill_server`) does each pick and place with the motion and gripper nodes.

Safety rule: the language model only proposes commands. The grammar and the main controller check every command. The model has no direct access to motion.

## 2. Hardware and workcell

| Item | Value |
|---|---|
| Robot arm | Fairino FR5, controller IP `192.168.58.2` |
| Gripper | Schmalz vacuum gripper on the tool flange. Tool output DO0 = vacuum pump, DO1 = blow-off pump. The tool tip is 107.4 mm from the flange. |
| Overhead camera | Logitech C920 (USB) above the table |
| Wrist camera | Intel RealSense D405 on a bracket next to the gripper (USB 3) |
| PC | Ubuntu 24.04 or newer, x86-64, NVIDIA GPU with 8 GB or more (tested: RTX 5060 Laptop, driver 595) |
| Network | Ethernet cable from the PC to the robot controller. PC address `192.168.58.10/24`. |

Board variants. All boards are 5 mm thick.

| Variant | Colour | Size |
|---|---|---|
| `VARa` | white | 295 x 210 mm (large) |
| `VARb` | white | 150 x 210 mm (small) |
| `VARc` | red | 135 x 210 mm |

Zones. Each zone is a sheet with an ArUco tag. Vision 1 finds the zones from their tags.

| Tag ID | Zone name | Kind |
|---|---|---|
| 2 | `processing_1` | processing (an operator works on the board) |
| 1 | `drop_off_a` | drop-off (stack) |
| 16 | `drop_off_b` | drop-off (stack) |
| none | `fault` | a fixed spot behind the robot base. The robot drops bad boards there. |

Boards that are not on a zone sheet are input boards. Jobs pick only input boards, except `clear` jobs.

## 3. Repository layout

```
pixi.toml, pixi.lock      the two pixi environments: default (ROS 2) and models (GPU models)
src/                      ROS 2 packages (motion, gripper, description, MoveIt, task controller, interfaces)
external/frcobot_ros2     the Fairino driver (git submodule, do not edit)
vision1/                  Vision 1 node, detector, table calibration, wrist camera launch file
models/boards-seg.onnx    the board segmentation model of Vision 1
samples/                  six camera images for the offline check of Vision 1
config/                   camera calibration, D405 settings, udev rule for the D405
conversation/             chat bridge, planner, grammar, model server, Gemma launcher
web/                      web app (Vite) and its HTTPS gateway
tools/                    stack.py, task_cli.py (tc), agent bridge, live monitor, vacuum demo page
tests/                    test scripts and tests/run.sh
notes/                    design records and decision logs
```

## 4. Install on a new workstation

Do these steps one time on each new PC.

### 4.1 Prerequisites

1. Install Ubuntu 24.04 or newer.
2. Install the NVIDIA driver. The driver must support CUDA 12.9 or newer. Run `nvidia-smi` to see the CUDA version of the driver.
3. Install git, curl and openssl:
   ```bash
   sudo apt install git curl openssl
   ```
4. Install pixi:
   ```bash
   curl -fsSL https://pixi.sh/install.sh | sh
   ```
5. Open a new terminal. Make sure that `pixi --version` shows a version.

Do not install project packages with `apt`, `pip` or `conda`. Pixi installs all project packages.

### 4.2 Get the code

```bash
git clone --recurse-submodules https://github.com/AtharvaBhorpe/fr5-board-stacking.git
cd fr5-board-stacking
```

If you cloned without `--recurse-submodules`, get the Fairino driver now:

```bash
git submodule update --init
```

### 4.3 Install the environments

```bash
pixi install --locked -a
```

This command installs two environments:

- `default`: ROS 2 Humble, MoveIt 2, the RealSense driver, OpenCV, ONNX Runtime with CUDA, Node.js.
- `models`: PyTorch with CUDA, llama.cpp with CUDA, Transformers, Phonon.

The download is about 8 GB. The two environments use about 14 GB of disk.

### 4.4 Build

```bash
pixi run build
```

This command builds the ROS packages with colcon and builds the web app into `web/dist`.
The first build shows the warning `Could not find activation scripts ... install/setup.sh`. This warning is expected. It does not show again.

Run `pixi run build` again after each change to a file in `src/` that is not Python. Python changes do not need a new build, because colcon uses symlinks. Run `pixi run build` again after each change in `web/src`.

### 4.5 Allow access to the D405 (real robot only)

```bash
sudo cp config/99-realsense-d405.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

Disconnect the D405 and connect it again.

### 4.6 Set up the robot network (real robot only)

1. Connect the Ethernet cable from the PC to the robot controller.
2. Find the name of the Ethernet interface with `ip link`. For example `enp3s0`.
3. Add a fixed address:
   ```bash
   nmcli con add type ethernet ifname enp3s0 con-name fr5-robot ipv4.method manual ipv4.addresses 192.168.58.10/24
   nmcli con up fr5-robot
   ```
4. Make sure that the robot answers:
   ```bash
   ping -c3 192.168.58.2
   ```

### 4.7 Download the models

The first start downloads the models into the Hugging Face cache (`~/.cache/huggingface`). Later starts work offline.

| Model | Size | Used for |
|---|---|---|
| Gemma 4 E2B QAT q4_0 (GGUF) | 4.3 GB | planner, questions about the scene, spoken replies |
| Parakeet TDT 0.6B v3 | 2.5 GB | speech to text |
| Phonon 40M multilingual | 0.1 GB | text to speech (English and German) |

To download the models before the first start, run each model process one time and stop it with Ctrl+C after it shows `READY`:

```bash
pixi run -e models llm        # Gemma, port 8770
pixi run -e models speech     # Parakeet + Phonon, port 8766
```

## 5. Start the simulation

The simulation needs no robot and no cameras. A fake arm, a fake gripper and a fake camera (`fake_world`) replace them.

```bash
pixi run up --sim
```

The launcher starts each process in order and waits until each part is ready.
If startup fails, the launcher stops its processes and returns a nonzero exit code.
During operation, the launcher reports process exits and keeps the remaining processes running.
A speech or web error does not switch off the gripper during a job.
The log files stay on disk. Run `pixi run logs` in another terminal to read them.
After you correct the problem, start the system again.

Keep the launcher terminal open. Ctrl+C stops the whole stack.
From another terminal in this checkout, `pixi run down` also stops the stack.
The launcher refuses a second start in the same checkout.
If another program uses a required HTTP port, startup fails before any stack process starts.

The robot does not move at start-up.
When all parts are ready, the launcher opens the web app on http://localhost:8088.
It also opens the vacuum-failure page on http://localhost:8765.
Use `--no-gui` to keep both pages closed.

The first start takes up to 4 minutes, because Gemma and Parakeet load into the GPU. Later starts take about 1 minute.

Options of `pixi run up`:

| Option | Effect |
|---|---|
| `debug` | Default. Our nodes log each step. All process output goes to log files. |
| `demo` | Normal logging. The monitor records less data. |
| `--sim` | Simulation. Sets `ROS_LOCALHOST_ONLY=1`, so the simulation cannot talk to a real robot stack on the network. |
| `--rviz` | Also starts RViz with the robot model and the planning scene. |
| `--no-gui` | Does not open a browser. The launcher stays in the current terminal. |

Each process writes to `live_logs/current/processes/<name>.log`:

| Name | Process |
|---|---|
| `monitor` | live monitor: warnings, errors, job progress |
| `motion` | MoveIt 2, Fairino driver (or fake arm), gripper node |
| `vision` or `fake` | Vision 1 (real) or `fake_world` (simulation) |
| `world` | world model |
| `skills` | skill server |
| `ctrl` | main controller |
| `bridge` | agent bridge (text commands in, results out) |
| `demo-ui` | vacuum-failure page, port 8765 |
| `llm` | Gemma on llama.cpp, port 8770 |
| `speech` | model server: Parakeet, Phonon, port 8766 |
| `chat` | chat bridge |
| `web` | web gateway, ports 8088 (HTTP) and 8443 (HTTPS) |
| `wrist` | D405 driver (real robot only) |

To follow all process logs, run `pixi run logs` in another terminal.
To follow one process, run `tail -F live_logs/current/processes/motion.log`, for example.
Ctrl+C in a log terminal stops only the log view.

## 6. Start on the real robot

WARNING: Do not start the real robot without a person at the hardware emergency stop. The robot moves fast. The software Stop button is not an emergency stop.

### 6.1 Before each start

1. Make sure that no simulation runs on this PC:
   ```bash
   pixi run down
   pgrep -af "move_group|ros2_control_node|fake_world"
   ```
   The second command must show nothing.
2. Open the robot web UI on http://192.168.58.2.
3. Set the robot to automatic mode (the circular arrows symbol, not the hand symbol).
4. Enable the robot. Make sure that the web UI shows no alarms.
5. Prepare the table:
   - Put the zone sheets on the table. The overhead camera must see each ArUco tag.
   - The drop-off sheets must be empty. The controller counts stacks only in memory and starts at 0.
   - Put the input boards 0.30 m to 0.80 m from the centre of the robot base. Boards nearer or farther are out of reach.
   - Keep each input board 3 cm or more away from each zone sheet.
   - An input stack can have 5 boards or fewer.
   - Keep the fault spot behind the robot base free.
6. Make sure that a person stands at the emergency stop.

### 6.2 Start

```bash
pixi run up demo
```

By default, the web app refuses all robot jobs on the real robot. The web app shows `Web control is simulation-only`. To let the web app send jobs to the real robot, start with `WORKCELL_REAL=1`:

```bash
WORKCELL_REAL=1 pixi run up demo
```

Without `WORKCELL_REAL=1`, use `pixi run tc <command>` in another terminal for jobs.

The gripper node switches both pumps off at start. If a board is on the suction cup at start, the board falls.

## 7. Use the web app

Open http://localhost:8088 on the PC. The web app shows:

- the chat (text and voice),
- the overhead camera image with the detected boards,
- the wrist camera image and the wrist depth image,
- a 3D view of the robot,
- the job queue and the controller state,
- the Stop, Pause and Resume buttons.

### 7.1 Connect a phone or a tablet

The phone must be on the same network as the PC.

1. On the PC, click **Connect a device**.
2. Download the CA certificate and copy it to the phone.
3. On the phone, install and trust the certificate.
   - iOS: install the profile in Settings > General > VPN & Device Management. Then enable full trust in Settings > General > About > Certificate Trust Settings.
   - Android: the menu to install a certificate is different for each vendor. Search for "CA certificate" in Settings.
4. On the PC, copy the paired link. Open the link on the phone.
5. On the phone, enable voice replies and allow the microphone.
6. Hold **Hold to talk** while you speak. Release the button to send. A recording stops after 20 seconds.

The paired link contains a private token. Do not give the link to other persons. The token stays the same after a restart. To make a new token, erase `web/.local/session.json` and restart.

The web app records audio on the device. The PC transcribes the audio. No audio goes to the internet.

### 7.2 Requests that you can give

Type a request or say it. The bridge sends fixed commands without change. The planner reads all other requests.

Single jobs:

```
move two variant B boards to drop off b
put all the red boards on drop off b
throw one small white board in the bin
process a red board and put it on drop off a
move everything from drop off a to drop off b
keep moving red boards to drop off a until I say stop
```

Many jobs with priorities in one request. The planner understands up to 16 jobs in one request.

```
Process 3 red boards to drop off A with high priority, then two big white boards to drop off B with medium priority, and scrap 3 small boards with low priority
```

This request gives these jobs. They run in this sequence:

1. `process VARc processing_1 drop_off_a 3 10`
2. `transfer VARa drop_off_b 2 5`
3. `transfer VARb fault 3 1`

German requests:

```
Bewege zwei kleine weisse Platten auf Ablage A und eine rote Platte in den Ausschuss
Verarbeite eine grosse Platte mit hoher Prioritaet und lege sie auf Ablage B
Lege alle grossen Platten auf Ablage B und eine kleine Platte auf Ablage A
```

Words that the planner knows:

| Meaning | Words |
|---|---|
| `VARa` | variant a, large, big, wide, grosse |
| `VARb` | variant b, small, narrow, kleine |
| `VARc` | variant c, red, rote |
| `fault` | fault zone, scrap, bin, trash, reject, waste, garbage, Ausschuss, Muell |
| count 0 (all boards) | all, every, everything, alle |
| count -1 (continuous) | keep going, continuously, until I say stop |
| priority 10 | high, urgent, asap, immediately, important, hoch, dringend, wichtig, sofort |
| priority 5 | medium, normal, mittel |
| priority 1 | low, whenever, when there is time, niedrig |
| `clear` (move the boards of a zone) | from, empty, clear, out of, von, leer, raeum |
| `process` | process, machine, treat, verarbeite, bearbeite |

If you give no priority, the priority is 0. If two jobs have the same priority, they run in the sequence of the request.

Control words. The bridge sends these words at once. The bridge never blocks them.

| You say | Effect |
|---|---|
| `stop` | Smooth stop now. Then the controller pauses. If the gripper did not hold a board yet, the vacuum pump goes off. |
| `pause` | The robot finishes the current skill, then waits. |
| `resume`, `continue` | The robot continues. If the robot held a board at the stop, it drops the board into the fault zone. |
| `cancel all`, `cancel batch` | Erases the queue. A held board goes to its place first. |
| `home` | The robot moves to the home pose. |
| `status` | The bridge reads the controller state aloud. |

A question that is not a job goes to Gemma with the camera images, for example `what do you see?`. Gemma answers with text. Gemma cannot move the robot.

### 7.3 Processing zone

A `process` job puts a board on `processing_1`. An operator then works on the board:

1. Wait until the arm leaves the processing zone.
2. Take the board away.
3. Keep the board away for 2 seconds or more. The camera must see the empty zone.
4. Put the board back inside the sheet.

The robot then picks the board and puts it on the drop-off zone of the job. If nobody takes a loaded board for 30 seconds, the controller shows an alert. The processing zone holds one board at a time. While a board is in processing, the robot does other jobs.

### 7.4 Vacuum-failure demonstration

Open http://localhost:8765.

- **Simulate vacuum failure**: arms the failure. The failure starts 1.3 s into the next carry. The board drops next to the stack. The robot goes home, finds the board as a new input board, and picks it again.
- **Now**: starts the failure at once. The gripper must hold a board.
- **Pause**, **Resume**, **Stop**: the same as `tc pause`, `tc resume`, `tc stop`.

WARNING: On the real robot, this page switches the real pumps. Use it only with a person at the emergency stop.

## 8. Use the command line

In another terminal, use `pixi run tc <command>`.
In the command table below, replace `tc` with `pixi run tc`.

If the simulation runs, set `ROS_LOCALHOST_ONLY=1` in each other terminal before you use `tc` or `ros2`. The simulation uses only localhost, and a terminal without this setting cannot find its nodes:

```bash
export ROS_LOCALHOST_ONLY=1
pixi run tc status
```

| Command | Effect |
|---|---|
| `tc help` | Shows all commands. |
| `tc status` | Shows the controller state. |
| `tc world` | Shows the boards and the zones. |
| `tc list` | Shows the jobs and their counters. |
| `tc wait j3` | Follows job `j3` until it ends. |
| `tc transfer VARa drop_off_a 2` | Moves 2 input `VARa` boards to `drop_off_a`. Count 0 = all. Count -1 = continuous. Variant `any` = any variant. |
| `tc transfer VARa drop_off_a 2 10` | The same with priority 10. |
| `tc process VARa processing_1 drop_off_a 1` | Puts 1 board on `processing_1`. After processing, puts it on `drop_off_a`. |
| `tc clear drop_off_a drop_off_b` | Moves the boards of `drop_off_a` to `drop_off_b`. |
| `tc board b7 drop_off_b` | Moves board `b7` to `drop_off_b`. |
| `tc pause`, `tc resume`, `tc stop`, `tc home` | Control. |
| `tc cancel j3` | Ends job `j3`. Use it to end a continuous job. |
| `tc prio j3 10` | Sets the priority of job `j3` to 10. |
| `tc clear_queue` | Erases all queued jobs. |
| `tc reset_zone drop_off_a` | Tells the world model that the operator emptied `drop_off_a` by hand. |
| `tc inject vacuum_loss` | Vacuum failure now (the same as **Now** on the demo page). |

Scripts can also write a command into the pipe of the agent bridge:

```bash
echo "transfer VARa drop_off_a 1" > live_logs/current/bridge/cmd
cat live_logs/current/bridge/events.txt     # results: "CMD ... -> OK j3" or "-> REJECTED <reason>"
cat live_logs/current/bridge/state.txt      # compact controller state
```

## 9. Stop the system

1. Wait until the robot is at home and the suction cup holds no board. Alternatively, run `tc pause` and wait.
2. Run:
   ```bash
   pixi run down
   ```
3. On the real robot: in the robot web UI, set manual mode and disable the robot.

`pixi run down` asks the launcher to stop its process groups in sequence.
The web app and chat stop first. The motion stack and monitor stop last.
The launcher sends SIGINT first. If a process does not exit, the launcher sends SIGTERM, then SIGKILL.
The launcher reports any process group that stays active. It does not stop unrelated processes.
Ctrl+C in the launcher terminal uses the same shutdown sequence.

`pixi run down` switches the pumps off. If a board is on the suction cup, the board falls.

## 10. Logs

Each start makes a new folder `live_logs/session_<date>_<time>_<pid>/`. The link `live_logs/current` points to the newest folder.

| Path | Content |
|---|---|
| `processes/<name>.log` | standard output and errors from each process |
| `ros/` | the log file of each ROS node |
| `monitor/main.jsonl` | controller state, task events, world changes, gripper state, tool position at 10 Hz, skill results |
| `monitor/vision_raw.jsonl` | each `/vision/boards` message |
| `monitor/world_raw.jsonl` | each `/task/world` message |
| `monitor/joints.jsonl` | each `/joint_states` message |
| `monitor/rosout.log` | the log lines of all nodes |
| `bridge/` | `cmd` pipe, `events.txt`, `state.txt` of the agent bridge |

The `live_logs/` folder is not in git. When the disk is full, erase old sessions by hand.

## 11. Tests

```bash
pixi run test          # offline tests, no ROS graph, no robot: under 1 minute
pixi run test all      # also the ROS tests with their own headless simulation: about 20 minutes
```

The script shows one line for each suite and `ALL PASSED` at the end. Each ROS suite uses its own `ROS_DOMAIN_ID` on localhost. A running `pixi run up` is not affected. Do not run `pixi run test all` while a simulation runs, because the CPU load makes timing checks fail.

Planner accuracy test. This test needs Gemma (`pixi run up` or `pixi run -e models llm`). It sends nothing to the robot.

```bash
pixi run python conversation/plan_check.py --runs 3
```

The result on 2026-10-28 was 81 of 81 (27 requests, 3 runs each), 0.5 s for each plan.

End-to-end check through the web app. This check needs `pixi run up --sim`. It moves only the simulated robot. It sends a scene question, a free-form plan, direct commands, pause, resume, an unsafe request, a voice recording, stop and cancel. It makes sure that the result of each step is correct.

```bash
ROS_LOCALHOST_ONLY=1 pixi run python conversation/demo_check.py
```

Web app tests. These tests need `pixi run up --sim` and Google Chrome at `/usr/bin/google-chrome`.

```bash
pixi run python web/tests/gateway.py      # token, origin check, request limits, HTTPS certificate
cd web && node tests/browser.mjs          # page loads, panels, 3D view, no script errors
```

Details of each suite: `tests/README.md` (controller) and `tests/README.motion.md` (motion and gripper).

## 12. Configuration files

| File | Content | Change it when |
|---|---|---|
| `config/calib.yaml` | overhead camera to table calibration (homography and lens term) | the overhead camera moves. Calibrate again. |
| `config/d405.yaml` | D405 driver settings (848 x 480, 15 Hz) | |
| `vision1/detect.py` (top) | table regions, colour limits, area limits | the light or the boards change |
| `vision1/table.py` (top) | board sizes, zone tags and zone sizes | you add a zone or change a sheet |
| `src/hk_fr5_task/config/world.yaml` | variants, table height, timers, position of the fault spot | the workcell changes |
| `src/hk_fr5_task/config/skills.yaml` | speeds, heights, press depth, blow-off time, home pose | the tool or the table changes |
| `src/hk_fr5_task/config/main.yaml` | retry policy, alert times, skill timeouts | |
| `src/hk_fr5_task/config/scenario_demo.yaml` | the simulated table for `--sim` and the tests | |
| `src/hk_fr5_description/config/tool_geometry.yaml` | length of the suction cup, D405 mount angle | the tool changes |
| `src/hk_fr5_motion/config/obstacles.yaml` | table height, fixed obstacles for MoveIt | the workcell changes |
| `src/hk_fr5_gripper/config/gripper.yaml` | robot IP, tool outputs, vacuum timeouts | |

## 13. ROS interfaces

| Name | Type | Notes |
|---|---|---|
| `/vision/boards` | `std_msgs/String` (JSON) | boards and zones from Vision 1, 5 Hz. Schema: `notes/vision1-changes.md`. |
| `/vision/image` | `sensor_msgs/Image` | overhead image with the detections |
| `/wrist/wrist_camera/color/image_raw`, `/wrist/wrist_camera/depth/image_rect_raw` | `sensor_msgs/Image` | D405 colour and depth |
| `/task/world` | `hk_fr5_task_interfaces/World` | tracked boards, zone states, stack counts |
| `/task/status` | `std_msgs/String` (JSON) | controller state, jobs, alerts |
| `/task/add_job`, `/task/list_jobs`, `/task/control` | services | job queue and control |
| `/task/skills/{pick,place,home,wait_clear,release_safe}` | actions | used by the controller |
| `/motion/*`, `/gripper/*`, `/scene/*` | actions and services | motion, gripper and planning scene. Reference: the files in `src/hk_fr5_interfaces/`. |
| `/conversation/input`, `/conversation/output`, `/conversation/control` | `std_msgs/String` | chat between the web gateway and the chat bridge |

HTTP ports on the PC:

| Port | Process |
|---|---|
| 8088 | web app (HTTP, localhost only) |
| 8443 | web app (HTTPS, for phones and tablets) |
| 8765 | vacuum-failure page |
| 8766 | model server (localhost only) |
| 8770 | Gemma, OpenAI-style API (localhost only) |

## 14. Troubleshooting

| Problem | Cause | Do this |
|---|---|---|
| `pixi run up` shows `not built` | `install/` or `web/dist/` does not exist | Run `pixi run build`. |
| `pixi run up` shows `A stack is already running` | an earlier start still runs in this checkout | Run `pixi run down`. |
| A readiness check shows `NOT ready` | startup failed and the launcher stopped its processes | Run `pixi run logs`. Read the process log. Correct the problem. Then start again. |
| Startup shows `Address already in use` | another process uses a required HTTP port | Stop the conflicting application. Then start again. |
| The launcher reports that a process exited | a stack process stopped unexpectedly | Read the named log file. Follow section 9 to stop the stack. Correct the problem. Then start again. |
| `No space left on device` during `pixi install` | the disk is full | Erase old `live_logs/` sessions. Run `pixi clean cache`. |
| Gemma is not ready after 180 s | the model download is slow, or the GPU memory is full | Read `processes/llm.log`. Run `nvidia-smi`. Stop other GPU programs. |
| `CUDA error: out of memory` in `llm.log` or `speech.log` | another program uses the GPU | Stop the other program. Gemma, Parakeet and Vision 1 need about 7 GB together. |
| The web app shows `Speech recognition is still loading` | Parakeet loads for about 20 s | Wait. Typed chat works in that time. |
| The web app shows `Web control is simulation-only` | real robot without `WORKCELL_REAL=1` | Use `pixi run tc`, or restart with `WORKCELL_REAL=1 pixi run up demo`. |
| `tc` shows `/task/list_jobs not available` in your own terminal | the simulation uses only localhost | Run `export ROS_LOCALHOST_ONLY=1` in that terminal. |
| The chat shows `The main controller is not running` | no `/task/status` for 3 s | Read `world.log`, `skills.log` and `ctrl.log` in `live_logs/current/processes/`. |
| The chat shows `The main controller refused ...` | the controller found an error in the job | Read the reason in the message. For example, a zone name that does not exist. |
| The planner made a wrong plan | the request had no clear variant, zone or count | Use the words in section 7.2. Name the variant, the zone and the count. |
| The motion log shows `robot hardware: MOCK` on the real robot | a simulation process still runs | Run `pixi run down`. Then do step 1 of section 6.1. |
| The motion log shows `机械臂SDK连接失败` (SDK connection failed) | no network to the robot, or another client is connected | Run `ping 192.168.58.2`. Close the other client. |
| Controller state HALTED `3 motion failures` or `release failed` | the robot or the gripper has a problem | Examine the robot and the gripper as the message says. Then run `tc resume`. |
| Controller state HALTED `the gripper holds something the world does not know` | a board is on the suction cup | Hold the board. Run `pixi run ros2 action send_goal /gripper/release hk_fr5_interfaces/action/Release "{blow_off_time: 1.0}"`. Then run `tc resume`. |
| A board is skipped (`PLAN_FAILED` or `NO_IK`) | the board is out of reach | Move the board into the ring 0.30 m to 0.80 m from the base. |
| A job waits with `processing_1 is LOADED` | the operator did not take the board | Do the steps of section 7.3. |
| The drop-off count is wrong after a restart | the controller counts only in memory | Empty the sheet. Run `tc reset_zone drop_off_a`. |
| A board shows as `UNKWN` | the model is not sure, or the size fits no variant | Make sure that the board lies flat and alone. |
| A zone is not found | the camera did not see its tag for 60 s | Make sure that the tag is visible, clean and flat. |
| The D405 log shows `RS2_USB_STATUS_ACCESS` | the udev rule is missing | Do section 4.5. |
| `lsusb` does not show the D405 | cable or port problem | Use a USB 3 port directly on the PC. |
| `NO_VACUUM` | the suction cup did not seal | Make sure that the board is flat. Make sure that the air line is connected. |
| The board stays on the cup after a place | the blow-off pulse is too short | Increase `blow_off_time` in `skills.yaml`. Listen for the blow-off sound. |
| The phone cannot open the HTTPS page | guest Wi-Fi isolates devices, or the PC address changed | Use a private network. If the address changed, erase `web/.local/server.crt` and restart. |
| Shutdown reports that processes did not stop | a process did not exit after the shutdown signals | Read the launcher terminal and the named process logs. Examine the remaining processes before a restart. |

## 15. Known limits

- The overhead camera cannot separate boards of the same colour that lie on each other. A red board on a red board can give the result `NOT_VERIFIED` after a place. The stack height stays correct.
- The world model ignores an `UNKWN` detection inside a zone, because it is usually a hand. A real board that Vision 1 reads as `UNKWN` inside a zone is not seen.
- Input picks use a slow contact search, because the system does not use the wrist depth for heights yet.
- The contact search presses about 20 mm into the board before the gripper reports the vacuum.
- If a high-priority job has no matching board, the robot goes home before it does lower-priority jobs.
- Gemma E2B is a small model. The planner corrects the common errors, but an unusual request can give a wrong plan. Read the plan in the chat before the robot starts.

## 16. Design records

The code comments use decision IDs. The IDs refer to these files:

| File | IDs |
|---|---|
| `notes/decision-log-task.md` | `TM-*` decisions of the main controller, `TM-O*` open items |
| `notes/control-loop.md` | `CL-*` design of the control loop |
| `notes/scheduling-example.md` | the scheduling example that `main_core_check.py` tests |
| `notes/vision1-changes.md` | the `/vision/boards` JSON contract (`C*`) |
| `notes/vision1-human-setup.md` | the table set-up for Vision 1 |
| `notes/decision-log-motion.md` | `H-*`, `W-*`, `G*`, `X-*`, `T*` decisions of the motion and gripper stack |
| `notes/implementation-ref.md` | `R-*` implementation notes and test results of the motion stack |

Git history keeps the earlier development branches: `feat/robot-motion-gripper`, `feat/task-main-controller`, `feat/vision1-overhead`, `feat/robot-conversation`.
