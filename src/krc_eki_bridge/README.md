# krc_eki_bridge

An EthernetKRL bridge for a KR C4 without RSI (route C in `docs/DESIGN.md`). It has the same ROS interface as
`krc_rsi_bridge`, and it carries the override only.

- Publishes `krc_interfaces/KrcState` on `/krc/state`.
- Subscribes to `krc_interfaces/OverrideCmd` on `/krc/override_cmd`; `seq` is the heartbeat.
- Applies the same safety rules as the RSI bridge (`krc_eki_bridge/safety.py`):
  - clamp to `[max(ov_min, ov_floor), ov_max]`;
  - heartbeat watchdog (`wd_s`);
  - linear ramp to `ov_default` over `fallback_ramp_s`.
- Never sends motion targets.

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
heartbeat ramp and the clamp, plus the template checks.

The controller templates in `krc_templates/` are **NOT RUN**. See `krc_templates/README.md` for the sources and the
rules that are still unverified.
