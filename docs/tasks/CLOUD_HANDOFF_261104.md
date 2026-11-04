# Cloud handoff, 2026-11-04: tasks T14, T13, T15

For Claude Code sessions on the web (one session per task). Each section below has the prompt to paste.

## State of the repository
- T10–T12 are merged (#30, #31, #32); `main` is green in CI. Nothing in `src/` has run with the controller or a
  camera.
- Inputs added on 2026-11-04:
  - #37: `legacy/realtime_cl_cage2_semi_261103/` (the cage-2 modules of 2026-11-03 for the fan program FAN1 and the
    semicircle program ARC2: geometry, gates, gate log, failure map; its `README.md` lists two known issues), the two
    print programs, and `data/fan1cl_20261103_222754/`, `data/arc2ol_20261103_235936/`;
  - the T15 inputs PR: the cage-2 controller side, rigs and bore sidecars in the same folder (README section
    "Cage 2 (T15)"), the CLp4 program, and `data/bduo_20261103_195120/` (bore-dual closed loop).

## Order: each task starts after the previous one is merged

| Task | Builds on | Order |
|---|---|---|
| T14 (#35) angle failure map + `src/path_geometry` | — | 1 |
| T13 (#36) angle-aware readout gates (fan, semi) | T14 | 2 |
| T15 (#38) cage 2: ELP-B and bore2 nodes, bore-led A_b1 with bore2 primary | T13 (same readout packages) | 3 |
| T16 (#39) fan / semicircle controller | T13, T15 | not yet: no task file |

## What the cloud cannot do (leave to the lab laptop)
- No camera, no controller, no GPU, no bore weights, no video. The widths in `data/` were computed live in the lab.
- Write the hardware checks into the PR description as a list for the lab; do not claim any hardware result.

## Rules for every session
- Read `CLAUDE.md`, `docs/DESIGN.md` and the task file first. `CLAUDE.md` rules apply: feature branch, PR to `main`
  naming the task, no attribution trailers, no personal names, addresses or absolute paths, safety invariants.
- Branch from the current `main`; do not merge.
- If ROS 2 cannot be installed in the session, run the pure-Python tests with `pytest` and rely on CI for colcon.

## Prompt for T14

```
In this repository, implement task T14 (docs/tasks/T14.md, issue #35): the angle failure map tool and the new
package src/path_geometry. Branch from the current main; do not merge. Read CLAUDE.md, docs/DESIGN.md,
docs/tasks/T14.md, legacy/realtime_cl_cage2_semi_261103/README.md and data/README.md first, then semi_geom.py,
fan_geom.py and semi_failure_map_261103.py in that legacy folder. Work on a branch t14-angle-map and open a PR to
main named "T14: angle failure map tool and path geometry". Keep the package free of ROS imports and test it with
pytest; CI must pass. Acceptance = the six tests of the task file: the geometry equals the legacy classes (floats
within 1e-9, ints and names exactly); on data/arc2ol_20261103_235936 the bore, ELP-row and ELP-frame tables equal the
legacy tool's output (floats within 1e-9, the rest exactly); the ELP summary is binned by the observed angle as the
task defines it. Open the PR; do not merge it. Claim no hardware result.
```

## Prompt for T13 (after T14 is merged)

```
In this repository, implement task T13 (docs/tasks/T13.md, issue #36): angle-aware readout gates for the fan and
semicircle patterns in width_readout_elp and width_readout_bore1, built on src/path_geometry (T14, merged). Branch
from the current main; do not merge. Read CLAUDE.md, docs/DESIGN.md, docs/tasks/T13.md,
legacy/realtime_cl_cage2_semi_261103/README.md and data/README.md first, then fan_geom.py, arc_geom.py, semi_geom.py
and bore1/bore1_rt.py in that legacy folder, and the current gates
(src/width_readout_elp/width_readout_elp/readout.py, src/width_readout_bore1/width_readout_bore1/gate.py). Work on a
branch t13-angle-gates and open a PR to main named "T13: angle-aware readout gates (fan, semi)". Keep the core free
of ROS imports; CI must pass. Acceptance = the five tests of the task file: on both sessions in data/ the port equals
the legacy gates run offline on the same RSI ring (reason and leg exactly, s within 1e-9 mm); the legacy offline
gates match the live logs as the task states (ELP every frame; bores at the t_done ring cut, s_tip within 0.1 mm,
every larger difference matched by an earlier cut); the gate log equals semi_gate_20261103_235936.csv within the
stated tolerances; the angle cap for 0-90 deg equals legacy; every existing golden test is unchanged. Open the PR; do
not merge it. List the hardware checks for the lab in the PR description; claim no hardware result.
```

## Prompt for T15 (after T13 is merged)

```
In this repository, implement task T15 (docs/tasks/T15.md, issue #38): cage 2, i.e. the ELP-B and bore2 width
readouts and the bore-led source selection (plan A_b1 with bore2 primary, law B on the +X lines) in cl_controller.
Branch from the current main; do not merge. Read CLAUDE.md, docs/DESIGN.md, docs/tasks/T15.md, docs/tasks/T11.md,
the section "Cage 2 (T15)" of legacy/realtime_cl_cage2_semi_261103/README.md and data/README.md first, then the
modules and line ranges that section names (rt_readout.py, bore1/bore2_rt.py, bore1/bore1_rt.py, rt_controller.py,
rsi_ring.py, run_cl.py). Work on a branch t15-cage2 and open a PR to main named "T15: cage 2 ELP-B / bore2 readouts
and bore-led A_b1". Keep the core free of ROS imports; CI must pass. Acceptance = the four tests of the task file:
every existing golden unchanged with elp_b and bore2 off; on data/bduo_20261103_195120 the ELP-B and bore2 gates equal
the legacy gates offline (exactly) and the live logs as the task states, and the controller replay equals the legacy
controller (OvCmd per step and per packet, source, law, kind, switch records; floats within 1e-9), with the legacy
replay compared with the live ctrl / ov_cmd / switch logs and every difference explained; one test per start refusal;
law B under the same clamp, floor, watchdog and forced latch as law A. If krc_interfaces changes, say so in the PR.
Open the PR; do not merge it. List the hardware checks for the lab in the PR description; claim no hardware result.
```
