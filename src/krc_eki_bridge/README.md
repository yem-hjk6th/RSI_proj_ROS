# krc_eki_bridge

An EthernetKRL bridge for a KR C4 without RSI (route C in `docs/DESIGN.md`). It has the same ROS interface as
`krc_rsi_bridge`, and it carries the override only.

- Publishes `krc_interfaces/KrcState` on `/krc/state`.
- Subscribes to `krc_interfaces/OverrideCmd` on `/krc/override_cmd`; `seq` is the heartbeat.
- Applies the same safety rules as the RSI bridge (`krc_eki_bridge/safety.py`):
  - clamp to `[max(ov_min, ov_floor), ov_max]`;
  - heartbeat watchdog (`wd_s`);
  - linear ramp to `ov_default` over `fallback_ramp_s`;
  - forced latch (below).
- Never sends motion targets.

## Forced latch

`cl_controller` sends `source: forced` when its speed check latches. The EKI bridge then behaves like
`krc_rsi_bridge` inside the print window (`OverrideSupervisor`, source `forced`):

- The first `OverrideCmd` with `source == "forced"` sets a latch. Only a restart of the bridge clears it.
- From the next send on, the override ramps linearly from the last value sent under control to `ov_default` over
  `fallback_ramp_s`, then holds there.
  - If a heartbeat ramp is already running, it simply continues.
  - With nothing sent under control yet, or `fallback_ramp_s: 0`, it jumps to `ov_default`.
- Later commands and heartbeats change nothing: no new control value, no restart of the ramp, no end of the latch.
- The clamp to `[max(ov_min, ov_floor), ov_max]` still applies to every value.
- The node logs one warning when the latch is set.

`test/test_same_as_rsi.py` feeds one timeline to both bridges and checks that they send the same override.

## Layout

| Module | ROS | Role |
|---|---|---|
| `safety.py` | no | `OverrideSupervisor`: clamp, floor, heartbeat ramp |
| `protocol.py` | no | `<State>` parsing, `<Ext>` message, TCP stream framing |
| `kinematics.py` | no | path speed (chord over `chord_s`) and `v_eff` (lag `tau_v`) |
| `core.py` | no | TCP client to the controller in its own thread; queues in and out, never blocks the ROS side |
| `fake_eki.py` | no | fake controller (EKI server and an `sps.sub` model) for tests and bench runs |
| `node.py` | yes | thin ROS 2 wrapper |

## KrcState over EKI

| Field | Source over EKI | When missing |
|---|---|---|
| `stamp` | PC receive time | – |
| `ipoc` | controller timer `T` [ms] (`$ROB_TIMER`, unverified) | 0 |
| `rist` | `$POS_ACT` X, Y, Z [mm], A, B, C [deg] | NaN |
| `path_speed_mm_s` | chord speed over `chord_s` (0.1 s), on the controller timer if present, else PC time | 0 |
| `v_eff_mm_s` | first-order lag of the path speed, τ 0.28 s | 0 |
| `override_pct` | `$OV_PRO` as reported by the controller | 0 |
| `vel_act_mm_s` | `$VEL_ACT` × 1000 | NaN |
| `rpm_ext` | `EffectiveRPM_mon` (extruder rpm command) | 0 |
| `late_replies` | always 0: EKI has no reply deadline | 0 |

The state rate is the controller's submit cycle (roughly 12 ms, unverified), not the 4 ms of RSI.

## Fallbacks on both ends

- Bridge: if the controller node stops sending fresh `seq` values, the bridge ramps to `ov_default`.
- Controller (`krc_templates/sps_eki_override.sub`): if no new `Seq` arrives for `EKI_LOSS_CYCLES` cycles, or the
  connection drops, `sps.sub` writes `EKI_OV_SAFE`. It writes `$OV_PRO` only in AUT/EXT.

## Run

```bash
ros2 launch krc_eki_bridge eki_bridge.launch.py params_file:=<PARAMS_FILE>
```

To test without a controller, start the fake controller first, then point the bridge at it with
`controller_host:=127.0.0.1`:

```bash
ros2 run krc_eki_bridge fake_eki_server --port 54600
```

## Tests

```bash
python3 -m pytest src/krc_eki_bridge/test
```

These are pure-Python tests against the fake EKI server. They cover state parsing, the override send, the
heartbeat ramp, the clamp and the forced latch, plus the template checks. `test_same_as_rsi.py` also imports
`krc_rsi_bridge` (from `src/` or the install space).

The controller templates in `krc_templates/` are **NOT RUN**. See `krc_templates/README.md` for the sources and the
rules that are still unverified.
