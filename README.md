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

**Status:** under development. Nothing in `src/` has been run against a real controller yet.

## Layout

| Path | Content |
|---|---|
| `docs/DESIGN.md` | Architecture, interfaces, safety invariants, test strategy |
| `src/` | ROS 2 packages (`krc_interfaces`, …) |
| `legacy/realtime_cl_261023/` | The Python pipeline used in the lab on 2026-10-22 (reference for the port) |
| `legacy/krc_config/` | Controller-side files of that pipeline: RSI config, sps.sub, one KRL program |
| `data/e1_20261022_165128/` | One real closed-loop print: RSI log, controller log, width readout, frame index, 40 camera frames |

## Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build && colcon test && colcon test-result --verbose
```

## Safety

- This code changes the speed of an industrial robot.
- Read `docs/DESIGN.md` §Safety before connecting it to a controller.
- Network addresses, frames and calibration belong in local configuration files, never in this repository.

## Licence

MIT. See `LICENSE`.
