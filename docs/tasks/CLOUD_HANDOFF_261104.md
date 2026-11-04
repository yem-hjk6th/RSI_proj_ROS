# Cloud handoff, 2026-11-04: tasks T14 and T13

For Claude Code sessions on the web (one session per task). Each section below has the prompt to paste.

## State of the repository
- T10–T12 are merged (#30, #31, #32); `main` is green in CI. Nothing in `src/` has run with the controller or a
  camera.
- Added on 2026-11-04 (inputs for the tasks):
  - `legacy/realtime_cl_cage2_semi_261103/`: the cage-2 modules of 2026-11-03 for the fan program FAN1 and the
    semicircle program ARC2 (geometry, gates, gate log, failure map); its `README.md` explains how they are used and
    lists two known issues;
  - `legacy/krc_config/krl/usr_ELP_FAN1_OL_261103.src`, `usr_ELP_ARC2_OL_261103.src`: the two print programs;
  - `data/fan1cl_20261103_222754/` (FAN1 closed loop, bore-led) and `data/arc2ol_20261103_235936/` (ARC2 open loop
    with every reading logged): logs of all four cameras, no video.

## Order and what blocks what

| Task | Blocks | Order |
|---|---|---|
| T14 (#35) angle failure map + `src/path_geometry` | T13 (geometry) | 1 |
| T13 (#36) angle-aware readout gates | FAN1 / ARC2 through the ROS readout nodes | 2: start when T14's branch has the geometry |

## What the cloud cannot do (leave to the lab laptop)
- No camera, no controller, no GPU, no bore weights, no video. The widths in `data/` were computed live in the lab.
- Write the hardware checks into the PR description as a list for the lab; do not claim any hardware result.

## Rules for every session
- Read `CLAUDE.md`, `docs/DESIGN.md` and the task file first. `CLAUDE.md` rules apply: feature branch, PR to `main`
  naming the task, no attribution trailers, no personal names, addresses or absolute paths, safety invariants.
- If ROS 2 cannot be installed in the session, run the pure-Python tests with `pytest` and rely on CI for colcon.
- Open the PR; do not merge it.

## Prompt for T14

```
In this repository, implement task T14 (docs/tasks/T14.md, issue #35): the angle failure map tool and the new
package src/path_geometry. Read CLAUDE.md, docs/DESIGN.md, docs/tasks/T14.md,
legacy/realtime_cl_cage2_semi_261103/README.md and data/README.md first, then semi_geom.py, fan_geom.py and
semi_failure_map_261103.py in that legacy folder. Work on a branch t14-angle-map and open a PR to main named
"T14: angle failure map tool and path geometry". Keep the package free of ROS imports and test it with pytest; CI
must pass. Acceptance = the six tests of the task file: the geometry equals the legacy classes (floats within 1e-9,
ints and names exactly); on data/arc2ol_20261103_235936 the bore, ELP-row and ELP-frame tables equal the legacy
tool's output (floats within 1e-9, the rest exactly); the ELP summary is binned by the observed angle as the task
defines it. Push the geometry commit early: task T13 branches from it. Open the PR; do not merge it. Claim no
hardware result.
```

## Prompt for T13 (when T14's branch has the geometry)

```
In this repository, implement task T13 (docs/tasks/T13.md, issue #36): angle-aware readout gates for the fan and
semicircle patterns in width_readout_elp and width_readout_bore1. It builds on src/path_geometry from T14 (#35); if
that is not on main yet, branch from t14-angle-map and say so in the PR. Read CLAUDE.md, docs/DESIGN.md,
docs/tasks/T13.md, legacy/realtime_cl_cage2_semi_261103/README.md and data/README.md first, then fan_geom.py,
arc_geom.py, semi_geom.py and bore1/bore1_rt.py in that legacy folder, and the current gates
(src/width_readout_elp/width_readout_elp/readout.py, src/width_readout_bore1/width_readout_bore1/gate.py). Work on a
branch t13-angle-gates and open a PR to main named "T13: angle-aware readout gates (fan, semi)". Keep the core free
of ROS imports; CI must pass. Acceptance = the five tests of the task file: on both sessions in data/ the port
equals the legacy gates run offline on the same RSI ring (reason and leg exactly, s within 1e-9 mm); the legacy
offline gates are compared with the live logs, with the agreement reported per camera and session and every
difference explained; the gate log equals semi_gate_20261103_235936.csv within the stated tolerances; the angle cap
for 0-90 deg equals legacy; every existing golden test is unchanged. Open the PR; do not merge it. List the hardware
checks for the lab in the PR description; claim no hardware result.
```
