# RSI_proj_ROS

ROS 2 (Jazzy) software for closed-loop robotic pellet extrusion on a KUKA KR 10 R1100-2 (KR C4, KSS 8.3, KUKA.RSI 3.3).

## How it works

- The KRL program on the KR C4 executes the toolpath.
- Every 4 ms the controller exchanges one RSI UDP packet with a PC.
- The PC reads the bead width from cameras that trail the nozzle.
- It closes the loop by setting the program speed override through RSI.
  - Position corrections stay at zero.

This repository ports that PC side, now a set of Python scripts (see `legacy/`), to ROS 2 packages. It adds:
- a simulator of the controller;
- an EthernetKRL (EKI) bridge, so the same nodes can drive a robot without RSI.

## Hardware status

**Nothing in `src/` has run against a real controller, camera or robot yet.** Every result so far comes from the
simulator (`krc_sim`), from replays of one logged print (`data/e1_20261022_165128/`) and from the golden tests against
the legacy code. The controller-side files are templates marked `NOT RUN`.

## Packages

| Package | One line |
|---|---|
| `krc_interfaces` | Messages: `KrcState`, `OverrideCmd`, `Width`, `ReplyRecord`, `CtrlRecord` (`docs/DESIGN.md` §5). |
| `krc_rsi_bridge` | RSI UDP server with its 4 ms reply loop in its own process; clamp, speed floor, heartbeat watchdog and fallback ramp. |
| `krc_sim` | Fake KR C4 RSI client: 4 ms packets, motion along a pattern at the replied override, late-reply counting with the RSI Timeout, replay of RSI logs. |
| `cl_controller` | The v3 control law as a pure library, and a node: `Width` + `KrcState` → `OverrideCmd`. |
| `width_readout_elp` | ELP chroma-edge bead-width readout: image + `KrcState` → `Width`. |
| `krc_eki_bridge` | The same topics over EthernetKRL TCP (route C); KRL/XML templates marked `NOT RUN`. |
| `krc_replay` | End-to-end replay of a logged session through the ROS graph, and the RSI link latency test. |
| `print_session` | One launch command for a print session: start-up checks, `session_meta.json`, rosbag2 recording, export to the legacy logs. |

## Layout

| Path | Content |
|---|---|
| `docs/DESIGN.md` | Architecture, packages, interfaces, safety invariants, test strategy |
| `docs/DEPLOY.md` | Edge PC (Ubuntu 24.04) and client deployment |
| `docs/deploy_wsl.md` | Edge PC as WSL 2 on the Windows 11 lab laptop (untested on hardware), latency test |
| `docs/tasks/` | Task specs of the GitHub issues |
| `src/` | ROS 2 packages |
| `config/e1.yaml` | The parameter set of session E1, one section per node |
| `legacy/realtime_cl_261023/` | The Python pipeline used in the lab on 2026-10-22 (reference for the port) |
| `legacy/krc_config/` | Controller-side files of that pipeline: RSI configurations (`RSI_OV4`, `RSI_MIN`), sps.sub, one KRL program |
| `data/e1_20261022_165128/` | One real closed-loop print: RSI log, controller log, width readout, frame index, 40 camera frames |

## Build and test

With Docker, as in CI (`.github/workflows/ci.yml`), from the repository root:

```bash
docker run --rm -it -v "$PWD":/ws -w /ws ros:jazzy-ros-base bash
```

Inside the container:

```bash
apt-get update && apt-get install -y python3-colcon-common-extensions python3-pytest python3-pandas
rosdep update && rosdep install --from-paths src --ignore-src -y
source /opt/ros/jazzy/setup.bash
colcon build && colcon test && colcon test-result --verbose
```

On a native ROS 2 Jazzy install, the last three lines are enough. The core of every package is plain Python with no
ROS imports; those tests also run with plain `pytest` (e.g. `python3 -m pytest src/krc_rsi_bridge/test`), and the ROS
tests skip themselves without a sourced workspace.

## What is tested, and how

- **Golden tests.** The legacy modules in `legacy/` were run on the inputs in `data/`, and their outputs are stored as
  fixtures. The ports must reproduce them: the control law's override sequence of E1 exactly (`cl_controller`), the
  ELP readout rows (`width_readout_elp`), the bridge's reply loop and path speed (`krc_rsi_bridge`), and the legacy
  eval on exported logs (`print_session`).
- **Replay** (`krc_replay`). E1 through the ROS graph: lock-step equivalence of the nodes with the library ports, and a
  real-time chain (`krc_sim` → bridge → controller → bridge) over a 59 s slice of the session, with a heartbeat-loss
  case. The whole session runs in the manual workflow `replay-full`.
- **Safety invariants** (`CLAUDE.md`, `docs/DESIGN.md` §6), each covered by tests:
  1. every RSI reply carries RKorr = 0 (`krc_rsi_bridge` reply format, the replay and latency reports);
  2. the override is clamped to `[ov_min, ov_max]` with the speed floor (`test_clamp.py`, replay, latency test);
  3. a stale heartbeat ramps linearly to `ov_default` over `fallback_ramp_s` (`test_watchdog.py`, replay test 5);
  4. the 4 ms reply loop runs in its own process and never blocks (`test_stall*.py`, timing tests on `krc_sim`);
  5. no motion targets are streamed;
  6. controller-side files are `NOT RUN` templates (`krc_eki_bridge` `test_templates.py`).
- **Latency test** (`krc_replay`, `docs/deploy_wsl.md` §4). `krc_sim` against a bridge on another host; CI runs it on
  loopback for 15 s.

## First hardware run

In this order, each step only after the previous one passed:

1. **Replay** of E1 on the edge PC (the `krc_replay` tests), and the latency test over the real cable.
2. **Shadow mode** (`mode:=shadow`) with a print program on **`RSI_MIN`**. In shadow mode the reply carries no OvCmd;
   on a program that uses `RSI_OV4` the OvCmd input is then 0, `LIMIT1` (1..100) raises it to 1, and **the robot runs
   at 1 % override**. `print_session` logs this warning at start-up.
3. **Air step test** (`mode:=step`, no material).
4. **Print** (`mode:=closed`).

## Safety

- This code changes the speed of an industrial robot.
- Read `docs/DESIGN.md` §6 before connecting it to a controller.
- Network addresses, frames and calibration belong in local configuration files, never in this repository.

## Licence

MIT. See `LICENSE`.
