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

| Package | Type | Ported from | Role |
|---|---|---|---|
| `krc_interfaces` | ament_cmake | – | Messages (§5) |
| `krc_rsi_bridge` | ament_python | `rsi_proc.py`, `rsi_ov_listener.py`, `rsi_ring.py` | RSI UDP server in its own process. Publishes `KrcState`, subscribes to `OverrideCmd`. Clamps, floor, heartbeat watchdog and fallback ramp live here. |
| `krc_sim` | ament_python | `replay_smoke.py`, `harness_run_cl.py` | Fake KR C4 RSI client: 4 ms packets, kinematics along a pattern, obeys the override, counts late replies like the controller. Also replays `rsi_data_*.csv`. |
| `cl_controller` | ament_python | `rt_controller.py`, `rt_law_v3.py` | Control law as a pure library, plus a node: `Width` + `KrcState` → `OverrideCmd` |
| `width_readout_elp` | ament_python | `rt_readout.py`, `rig/`, `pattern/` | ELP chroma-edge width readout: image + `KrcState` → `Width` |
| `krc_eki_bridge` | ament_python | – | Same topics as the RSI bridge, over EthernetKRL TCP. KRL/XML templates marked NOT RUN. |
| `print_session` | ament_python | `run_cl.py` | Session lifecycle, parameters, rosbag2 recording (later) |
| `width_readout_bore1` | ament_python | `bore1/` | GPU head; weights are not in this repository (later) |

## 5. Interfaces (contract; change only through a PR to `krc_interfaces`)

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
- `float64 s_tip_mm` (nozzle position along the leg at the frame time, so `s_mm = s_tip_mm - d_mm`; NaN when the publisher does not know it. A message field cannot default to NaN, so publishers always set it.)
- `float64 width_mm`
- `bool valid`

**Topics**: `/krc/state`, `/krc/override_cmd`, `/width/elp`, `/width/bore1`.

## 6. Safety invariants

See `CLAUDE.md`. In short:
- zero position correction;
- override clamped to `[ov_min, ov_max]` with the configured floor;
- heartbeat watchdog with a linear fallback ramp;
- a reply loop that never blocks;
- no motion streaming;
- controller-side files are NOT RUN templates.

On hardware, the first runs use the legacy order: shadow mode (computed but not sent), then an air step test, then printing.

## 7. Tests

- **Golden tests** against `legacy/` on `data/e1_20261022_165128/`. The ported law must reproduce the logged override sequence when fed the logged inputs.
- **Timing tests** on `krc_sim`: reply latency distribution, and late-reply counting with a 100-packet timeout.
- **CI:** colcon build and test in a `ros:jazzy` container on every push.
