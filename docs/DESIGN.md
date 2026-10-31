# Design

## 1. Starting point

**The cell today**
- KUKA KR 10 R1100-2, KR C4, KSS 8.3, KUKA.RSI 3.3.
- Pellet extruder: a motor speed command from `sps.sub` to an analog output.
- Two width sensors:
  - ELP: a global-shutter USB camera trailing the nozzle. It reads the settled bead 12–25 mm behind the nozzle.
  - bore1: a USB borescope at the nozzle exit with a learned width head.

**What the PC side does now** (`legacy/realtime_cl_261023/`, Windows, Python)
- It answers every RSI packet within the 4 ms cycle: echoes IPOC, RKorr = 0, sends `OvCmd`.
- It reads width from the cameras, using the RSI pose for the arc-length position.
- It runs a control law that maps width error to a speed override.

**RSI timing constraint**
- If replies are late, the controller counts them. When the counter reaches the Timeout parameter (100), RSI shuts down.
- In one lab run a 2 s stall in the Python process gave 355 late replies in a row. The reply loop has lived in its own process since then (`legacy/realtime_cl_261023/rsi_proc.py`).

## 2. Routes

| Phase | Route | Controller-side change |
|---|---|---|
| now | **A**: ROS 2 replaces the PC side; same KRL, same RSI config (`legacy/krc_config/rsi/`) | none |
| now (templates only) | **C**: the same ROS interfaces over EthernetKRL; `sps.sub` writes `$OV_PRO` (allowed only in AUT/EXT) | EKI XML + sps.sub edit, later |
| later | **B**: ROS streams motion over RSI. Precedents are kroshu/kuka_drivers (tested from KSS 8.6.11 / RSI 4.1.3) and fzi kuka_rsi_driver. | new RSI network interface and config; out of scope here |

## 3. Deployment: server and client

- **Edge PC** (wired to the KR C4 and to the cameras): Ubuntu 24.04, ROS 2 Jazzy, low-latency or PREEMPT_RT kernel. It runs the bridge, cameras, readout, controller and recorder. Every loop with a deadline closes here.
- **Client** (LAN or Wi-Fi): visualisation, parameters, start/stop, bag playback.
  - It is never in the control path.
  - On Wi-Fi use a Discovery Server or `rmw_zenoh`, and compressed image transport.
- **RSI never runs over Wi-Fi.**

## 4. Packages

Status: **implemented** = merged and tested in CI against the simulator and the logged session E1; none of it has
run on hardware. **planned** = not in the repository yet.

| Package | Type | Ported from | Role | Status |
|---|---|---|---|---|
| `krc_interfaces` | ament_cmake | – | Messages (§5) | implemented |
| `krc_rsi_bridge` | ament_python | `rsi_proc.py`, `rsi_ov_listener.py`, `rsi_ring.py` | RSI UDP server in its own process. Publishes `KrcState` and `ReplyRecord`, subscribes to `OverrideCmd`. Clamps, floor, heartbeat watchdog and fallback ramp live here. | implemented |
| `krc_sim` | ament_python | `replay_smoke.py`, `harness_run_cl.py` | Fake KR C4 RSI client: 4 ms packets, kinematics along a pattern, obeys the override, counts late replies like the controller. Also replays `rsi_data_*.csv`. | implemented |
| `cl_controller` | ament_python | `rt_controller.py`, `rt_law_v3.py` | Control law as a pure library, plus a node: `Width` + `KrcState` → `OverrideCmd`; `CtrlRecord` after every update | implemented |
| `width_readout_elp` | ament_python | `rt_readout.py`, `rig/`, `pattern/` | ELP chroma-edge width readout: image + `KrcState` → `Width` | implemented |
| `krc_eki_bridge` | ament_python | – | Same topics as the RSI bridge, over EthernetKRL TCP. KRL/XML templates marked NOT RUN. | implemented (templates NOT RUN) |
| `krc_replay` | ament_python | – | End-to-end replay of a logged session through the ROS graph: `log_player` node (lock-step and real-time), equivalence and safety tests. Also the RSI link latency test (`latency_test` CLI, `latency_bridge.launch.py`). | implemented |
| `print_session` | ament_python | `run_cl.py` | Session lifecycle: one bringup launch with the start-up checks and refusals of `run_cl.py`, `session_meta.json`, rosbag2 recording (mcap) of the record topics, `bag_to_legacy` export to the legacy CSV logs. Warns at start-up in shadow mode (no OvCmd: 1 % on `RSI_OV4`). | implemented |
| `width_readout_bore1` | ament_python | `bore1/` | GPU head; weights are not in this repository | planned |

## 5. Interfaces (contract; change only through a PR to `krc_interfaces`)

All five messages are implemented.

**`KrcState.msg`**: one per RSI packet.
- `builtin_interfaces/Time stamp` (PC receive time)
- `uint64 ipoc`
- `float64[6] rist` (X, Y, Z mm; A, B, C deg, as sent)
- `float64 path_speed_mm_s` (chord over the last 25 packets)
- `float64 v_eff_mm_s` (first-order lag of path speed, τ 0.28 s)
- `int32 override_pct`
- `float64 vel_act_mm_s`
- `int32 rpm_ext` (motor rpm command)
- `uint32 late_replies`

**`OverrideCmd.msg`**:
- `builtin_interfaces/Time stamp`
- `int32 value_pct`
- `string source` (`elp`, `bore1`, `model`, `forced`, `default`)
- `uint64 seq`

**`Width.msg`**:
- `builtin_interfaces/Time stamp` (frame time)
- `string source`
- `int32 leg`
- `float64 s_mm` (arc length along the leg)
- `float64 d_mm` (distance behind the nozzle)
- `float64 width_mm`
- `bool valid`

**`ReplyRecord.msg`**: one per RSI packet, published by `krc_rsi_bridge` from its packet ring after the reply was
sent. A row of the legacy `ov_cmd_<ses>.csv`, plus the packet fields `KrcState` does not carry.
- `builtin_interfaces/Time stamp` (the same value as the packet's `KrcState.stamp`)
- `float64 pc_ts_perf` (receive time on `CLOCK_MONOTONIC`)
- `uint64 ipoc`
- `int32 ov_cmd` (decided for the packet, after the clamp)
- `string source` (`outside`, `forced`, `step`, `default`, `control`, `watchdog`, `fallback`)
- `bool ov_sent` (false in shadow mode: the zero reply)
- `int32 delay` (the packet's `<Delay D>`)
- `float64 reply_latency_s` (packet arrival to reply sent)
- `string win` (bridge print window: `before`, `inside`, `after`)

**`CtrlRecord.msg`**: one per law update (every 0.25 s while the controller runs), published by `cl_controller`. A
row of the legacy `ctrl_<ses>.csv` (`CTRL_COLS` of `run_cl.py`, same names and order), unrounded; NaN or -1 where the
legacy log has an empty cell. Among its fields: `stamp`, `t_perf` (same clock as `ReplyRecord.pc_ts_perf`), `n_upd`,
`win`, `leg`, `s_tip`, `w_est`, `ov_raw`, `ov_pub` (the value handed to the bridge), `src`, `plan`, `state`, the
per-source validity and lag, the law terms (`e`, `du`, `u`, `lnk_*`) and the counters `n_corr`, `n_switch`. The full
list is in `src/krc_interfaces/msg/CtrlRecord.msg`.

**Topics**:

| Topic | Type | Publisher | Status |
|---|---|---|---|
| `/krc/state` | `KrcState` | `krc_rsi_bridge`, `krc_eki_bridge`, `krc_replay` `log_player` (lock-step) | implemented |
| `/krc/override_cmd` | `OverrideCmd` | `cl_controller` | implemented |
| `/krc/reply_record` | `ReplyRecord` | `krc_rsi_bridge` | implemented |
| `/controller/ctrl_record` | `CtrlRecord` | `cl_controller` | implemented |
| `/width/elp` | `Width` | `width_readout_elp`, `log_player` | implemented |
| `/width/bore1` | `Width` | `width_readout_bore1` (planned), `log_player` | replay only |

`print_session` records `/krc/state`, `/krc/override_cmd`, `/krc/reply_record`, `/controller/ctrl_record` and
`/width/*`.

## 6. Safety invariants

See `CLAUDE.md`. In short:
- zero position correction;
- override clamped to `[ov_min, ov_max]` with the configured floor;
- heartbeat watchdog with a linear fallback ramp;
- a reply loop that never blocks;
- no motion streaming;
- controller-side files are NOT RUN templates.

On hardware, the first runs use the legacy order: replay, then shadow mode (computed but not sent), then an air step
test, then printing. **Shadow mode needs a print program on `RSI_MIN`:** its reply carries no OvCmd, and on `RSI_OV4`
the missing input becomes 0, which `LIMIT1` (1..100) raises to 1 % override (`legacy/README.md`).

## 7. Tests

- **Golden tests** against `legacy/` on `data/e1_20261022_165128/`. The ported law must reproduce the logged override sequence when fed the logged inputs.
- **Timing tests** on `krc_sim`: reply latency distribution, and late-reply counting with a 100-packet timeout.
- **Replay** (`krc_replay`): lock-step equivalence of the nodes with the library ports, and the real-time chain on a
  slice of E1, with a heartbeat-loss case.
- **Latency test** (`krc_replay`): `krc_sim` against a bridge on another host; on loopback in CI.
- **CI:** colcon build and test in a `ros:jazzy` container on every push.
