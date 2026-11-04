# width_readout_bore1

bore1 bead-width readout (task T11), ported from `legacy/realtime_cl_261023/bore1/` (`bore1_head.py`, `bore1_rt.py`).
The borescope at the nozzle exit sees the bead 5 mm behind the nozzle; a learned head (three frozen ConvNeXt-T seeds)
turns each frame into a width. This package gates the frames on the RSI track, runs the head and publishes the width
for `cl_controller` (plan `A_b1`: bore1 → ELP → model, the plan of E1).

**Nothing here has run with a camera, a GPU or the real weights.** The weights (`cnxt_t_deploy_s{0,1,2}.pt`,
3 × 111 MB) are not in this repository; CI runs everything with the `replay` head. The laptop check is
`test/laptop_golden_bore1.py` (below).

The core has no ROS imports. `node.py` is a thin wrapper.

| Module | Ported from | Content |
|---|---|---|
| `gate.py` | `bore1_rt.bore1_gate` | The frame gate on the RSI track: `stale`, `no_track`, `z_or_dir` (`\|z − z_cmd(x)\| < 0.5`, moving −X: `x(t) − x(t − 0.1) < −0.3`), `leg_or_x` (`\|y − leg y\| < 0.5`, `770 < x < 945`), else `ok` with leg and `s_tip`. Uses `width_readout_elp`'s `Pattern` and `RsiRing` (ports of the same legacy classes). |
| `head.py` | `bore1_head.py` | Head interface, `Bore1Pred`, `crop288`, `QualityGate`, the weight check, `ReplayHead`, `make_head`. |
| `torch_head.py` | `bore1_head.Bore1Head` | The torch head: 3 seeds, bf16 autocast, one CUDA graph per frame. Imported only for `head: torch`. |
| `readout.py` | `Bore1Thread.run` | Row logic: gate → head → `s_dep = s_tip − 5`, `use = (q_ok or not drop_bad_q) and sd ≤ sd_max` → one bore1_rt row (legacy formats) and one `Width` row. |
| `replay.py`, `cli.py` | – | Offline replay of a logged session with the replay head; `width_readout_bore1_offline` writes a bore1_rt csv. |
| `record.py`, `params.py` | – | Row ↔ message fields; the node's parameters (no ROS). |
| `node.py` | – | `/bore1/image_raw` + `/krc/state` → `/width/bore1` (`Width`) and `/width/bore1/record` (`Bore1Record`). |

Fan and semicircle patterns (T13): `gate.make_gate` returns `path_geometry.gates.bore_gate` for a pattern json with
`"geometry"` `fan` or `semi` (below).

`config/weights_sha256.json` and `config/quality_limits.json` are byte-identical copies of the legacy files (tested).

## Heads

- **`torch`**: the legacy `Bore1Head`, loaded from `weight_dir`. Before torch is imported, every seed file is checked
  against the sha256 list committed in this package; a list next to the weights must be that same list (the legacy
  checked only against the list next to the weights). A missing file or a mismatch refuses the start; there is no
  switch to skip the check. Needs torch (a CUDA build; RTX 50 series: the cu128 wheels) and timm
  (`convnext_tiny.dinov3_lvd1689m`, `pretrained=False`), from pip, as `legacy/realtime_cl_261023/bore1/requirements_bore1.txt`
  says. They are not rosdep keys and not package dependencies.
- **`replay`**: the head outputs logged in a `bore1_rt_<ses>.csv(.gz)`, looked up by frame time (within
  `replay_tol_s`, 1 µs). A frame without a logged output gets NaN and `q_ok` false, so it is not pushed (counted as
  a miss). The image is not decoded. For tests and offline replays.

## Node

```bash
ros2 run width_readout_bore1 width_readout_bore1_node --ros-args --params-file src/width_readout_bore1/config/width_readout_bore1.yaml
```

Usually started by the bringup: `print_session.bore1_readout: true` with a `width_readout_bore1` section in the
session's parameter file (`print_session/README.md`).

| Topic | Type | |
|---|---|---|
| `/bore1/image_raw` (in) | `sensor_msgs/Image` | from `camera_capture` (T10); best effort, depth 1: frames that arrive while a frame is processed are dropped, as the legacy thread skipped them |
| `/krc/state` (in) | `KrcState` | pushed into the RSI ring at its stamp |
| `/width/bore1` | `Width` | one per frame that passed the gate: `d_mm` 5, `s_mm` = `s_tip − 5`, `s_tip_mm`, `width_mm`, `valid` = pushed (`q_ok` and `sd ≤ sd_max`). `cl_controller` ignores `valid = false`. |
| `/width/bore1/record` | `Bore1Record` | one per processed frame, also when the gate rejects it: the bore1_rt row, unrounded |

**Frame time** (`frame_time`). The legacy frame time was the PC receive time of the frame (`perf_counter` after
`cap.read()`), and the head's 5 mm label offset was fitted on it, so no latency is subtracted (unlike the ELP's
`lat_s`). `receive` (default) takes the time the image reached the node. `header` takes `header.stamp +
header_offset_s`: `camera_capture` stamps the V4L2 capture time, earlier than the read by the camera's
`measured.stamps.read_minus_capture_ms` (its `session_meta.json` block), so `header` with that offset is the legacy
definition without the ROS transport delay. Which one is closer on the laptop is a lab check.

Before the gate the node waits up to `wait_s` (0.06 s, as the legacy thread) for an RSI packet at or after the frame
time. A head that raises is logged and counted, and the frame publishes nothing (legacy: `errors`, no row).

With `session_meta` set (the bringup sets it), the node writes a `bore1_readout` block (head, frame time, counts, head
time mean / p95 / max: the legacy `bore1_shadow` summary; and `receive_minus_stamp_ms`, the node's receive time minus
the image stamp, for the frame-time check) at start and at stop.

Parameters (defaults in `params.py`, example `config/width_readout_bore1.yaml`): `camera`, the four topics,
`pattern_json` (empty = `width_readout_elp`'s packaged R1v30_CLp pattern), `root`, `head`, `weight_dir`,
`quality_json`, `device`, `mem_frac`, `warmup`, `replay_csv`, `replay_tol_s`, `frame_time`, `header_offset_s`,
`wait_s`, `drop_bad_q`, `sd_max`, `session_meta`, `bore_max_deg`, `bore_angle_at`, `gate_log_dir`, `lockstep`.

`lockstep: true` is a hook for replay tests, like `cl_controller`'s: inputs are queued, and a
`std_msgs/Float64MultiArray` on `/width_readout_bore1/step` (`[n_state, n_image]`) applies the first `n_state`
`KrcState` and then processes the images up to `n_image`, frame time = `header.stamp + header_offset_s`.

## Fan and semicircle patterns (T13)

A pattern json with `"geometry"` `fan` (FAN1) or `semi` (ARC2) switches the gate to `path_geometry.gates.bore_gate`
(kind A), the legacy `make_fan_bore_gate("A", max_deg)` / `make_semi_bore_gate("A", max_deg)` that `run_cl.py`
installs (`legacy/realtime_cl_cage2_semi_261103/`). Every other pattern runs `bore1_gate` unchanged.

- Tip at the frame time (no latency): `stale`, `no_track`, `z_or_dir` (`|z - z_cmd| >= 0.5`, or motion over 0.1 s along
  the local tangent `<= 0.3` mm), `leg_or_x` (no A line, `|e| >= 0.5`, or x outside `770 < x < 945`), `angle`
  (`|delta| > bore_max_deg + 1e-9`), else `ok`. `s_tip` is the arc length along the line, `s_dep = s_tip - 5`.
- `bore_max_deg` (default 0.0, as `run_cl.py --fan-bore-max-deg`; both 2026-11-03 sessions ran with 90). At 0 every
  frame off a 0-deg line is rejected: on ARC2 that is nearly every lobe frame.
- `bore_angle_at`: `tip` (default, legacy: the angle at `s_tip`) or `obs` (at `s_tip - 5`, where the bore observes;
  legacy README known issue 2). In the first 5 mm of a leg `s_tip - 5 < 0` and the angle there is the start tangent's
  (`BoreGate.obs_before_start`; 78 passed frames on ARC2-OL).
- Semi patterns: the gate log `semi_gate_<camera>.csv` (legacy columns, cam `bore1`, `s_obs = s_tip - 5`) goes to
  `gate_log_dir`, or to the folder of `session_meta` when that is empty; no log without either.
- `pattern_json` may be a file name in `path_geometry/patterns/`.

```bash
ros2 launch width_readout_bore1 width_readout_bore1.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json \
  weight_dir:=<BORE1_WEIGHT_DIR> bore_max_deg:=15.0 session_meta:=<SESSION_DIR>/session_meta.json
```

## Offline replay

```bash
ros2 run width_readout_bore1 width_readout_bore1_offline --session-dir data/e1_20261022_165128 --out bore1_rt.csv
```

The online ring was filled from the RSI process through a queue; which packets it held at each gate is not logged.
The replay gates frame k with the packets received at or before `t_gate = t_done − ms / 1000` of its logged row
(`replay.py`).

## Tests

| Test | |
|---|---|
| `test_golden.py` | Acceptance 1. Port vs the legacy `Bore1Thread` (fixture `test/fixtures/golden_legacy_e1_bore1.csv.gz`, regenerate with `python3 src/width_readout_bore1/test/make_golden.py`) on E1's frame times and RSI log with the replay head: all 12878 rows, gate / leg / pushed equal, `s_tip`, `s_dep`, `x`, `y`, `z` within 1e-6 mm, every written column equal as a string; the fixture is regenerated in the test and compared. Online check against `bore1_rt_*.csv.gz`: 202 rows differ, each explained (184: the online ring lacked the newest packet, received ≤ 2.2 ms before the gate; 18: one unit of the last logged decimal). The counts equal E1's `bore1_shadow` summary. |
| `test_node_ros.py` | Acceptance 2. Lock-step replay of all of E1 through the node: every `Bore1Record` and `Width` equal to the port's. Normal mode on a synthetic track (both frame times); refusals. |
| `test_controller_e1.py` | Acceptance 3. E1 through `cl_controller` (plan A_b1) with these `Width` rows: the published OvCmd equals the online run at 1718 of 1719 updates; update 796 is 1 % higher, explained (a drain-lag row moves the bore1 window edge past one bin). |
| `test_gate.py`, `test_head.py` | The gate at every limit vs the legacy `bore1_gate`; replay head, weight check, crop and quality terms vs the legacy, row logic. |
| `test_path_gate.py` (T13) | Both 2026-11-03 sessions replayed through `Bore1Readout` with the path gate (replay head, `replay.py` schedule): every row's gate, leg and pose equal `path_geometry.gates.bore_gate`, gate and leg equal the live log on every frame; the angle cap and `angle_at`; the semi gate log; `load_pattern` / `make_gate`. |
| `test_launch_fan.py` (T13) | `ros2 launch` with FAN1 by name, cap 20 deg: `ok` on the 0-deg lead line, `angle` on the -30-deg line 3. |
| `laptop_golden_bore1.py` | Acceptance 5, lab laptop only (not collected by `colcon test`; skips without weights, torch / timm or CUDA): port vs legacy `Bore1Head` on the same decoded frames, `w` within 1e-4 mm, head time p95 next to E1's 13.3 ms. |

The `print_session` exporter test (acceptance 4) is in `print_session/test/test_roundtrip.py`.

```bash
python3 -m pytest src/width_readout_bore1/test                   # with a sourced workspace for the ROS tests
BORE1_WEIGHTS=<weight dir> BORE1_VIDEO=<bore_*.mp4> python3 -m pytest src/width_readout_bore1/test/laptop_golden_bore1.py
```
