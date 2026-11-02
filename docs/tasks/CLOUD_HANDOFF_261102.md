# Cloud handoff, 2026-11-02: tasks T10, T11, T12

For Claude Code sessions on the web (one session per task). Each section below has the prompt to paste.

## State of the repository
- Everything in `src/` is merged and green in CI. Nothing has run with the controller or a camera.
- The lab laptop (the planned edge PC) now runs ROS 2 Jazzy in WSL 2: `docs/deploy_wsl.md` §7. All 358 tests ran
  there; the one failure was the ROS discovery problem of §7.1, fixed with `FASTDDS_BUILTIN_TRANSPORTS=SHM`.
- Added on 2026-11-02 (inputs for the tasks):
  - `src/cl_controller/patterns/`: the patterns of the programs on the controller (R1v_261016 for the air step test,
    E5, r240, R1v40_OL, R1v12_CL, the corner programs CNh, CNi, CNip, CNj, CNjp);
  - `legacy/pipeline_cage1012/`: `elp.py`, `borescope.py`, `config.py`, `pipeline.py` (camera capture and recording);
  - `legacy/realtime_cl_261023_ml/`: the walls variant (changed modules and wall patterns);
  - `legacy/realtime_cl_261023/bore1/weights/`: the weight sidecar JSONs and sha256 list (no `.pt`);
  - `legacy/krc_config/krl/usr_OV_step_261020.src`: the air step test program.

## Order and what blocks what

| Task | Blocks | Priority |
|---|---|---|
| T10 camera capture and recording | any print through ROS that keeps its video (the width dataset needs it) | 1 |
| T11 bore1 readout | plan `A_b1` | 2 |
| T12 walls | the wall programs `W4L28_*` | 3, after T10 and T11 are merged |

The air step test (`mode:=step`) and an open-loop print without video need none of them.

## What the cloud cannot do (leave to the lab laptop)
- No camera, no controller, no GPU, no bore1 weights (3 × 111 MB, kept on the laptop).
- Write the hardware checks into the PR description as a list for the lab; do not claim any hardware result.

## Rules for every session
- Read `CLAUDE.md`, `docs/DESIGN.md` and the task file first. `CLAUDE.md` rules apply: feature branch, PR to `main`
  naming the task, no attribution trailers, no personal names, addresses or absolute paths, safety invariants.
- If ROS 2 cannot be installed in the session, run the pure-Python tests with `pytest` and rely on CI for colcon.

## Prompt for T10

```
In this repository, implement task T10 (docs/tasks/T10.md): camera capture and recording.
Read CLAUDE.md, docs/DESIGN.md, docs/deploy_wsl.md section 7 and docs/tasks/T10.md first, then the legacy
references the task lists (legacy/pipeline_cage1012/elp.py, borescope.py, config.py and
legacy/realtime_cl_261023/elp_hevc.py). Work on a branch t10-camera-capture and open a PR to main named
"T10: camera capture and recording". Keep the core free of ROS imports and test it with pytest; CI must pass.
There is no camera or GPU in this session: put the hardware checks of the task into the PR description as a list
for the lab, and claim no hardware result.
```

## Prompt for T11

```
In this repository, implement task T11 (docs/tasks/T11.md): the bore1 width readout
width_readout_bore1. Read CLAUDE.md, docs/DESIGN.md and docs/tasks/T11.md first, then
legacy/realtime_cl_261023/bore1/ (bore1_head.py, bore1_rt.py) and the E1 data under data/e1_20261022_165128/.
The weights (.pt) are not in the repository: build the head interface with the torch and replay implementations,
make the CI golden tests use the replay head, and write the laptop script that skips without weights or CUDA.
The task adds krc_interfaces/Bore1Record.msg: say in the PR that it changes the interface contract.
Work on a branch t11-bore1-readout and open a PR to main named "T11: bore1 width readout". CI must pass.
Claim no hardware result.
```

## Prompt for T12 (only after T10 and T11 are merged)

```
In this repository, implement task T12 (docs/tasks/T12.md): the multi-layer (walls) variant.
Read CLAUDE.md, docs/DESIGN.md, docs/tasks/T12.md and legacy/realtime_cl_261023_ml/README.md first. Port the
changes listed there without changing the single-layer behaviour (every existing golden test must stay green).
Work on a branch t12-walls and open a PR to main named "T12: multi-layer walls". CI must pass.
Claim no hardware result.
```
