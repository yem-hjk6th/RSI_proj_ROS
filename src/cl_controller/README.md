# cl_controller

The v3 closed-loop bead-width control law (`legacy/realtime_cl_261023/rt_law_v3.py` and `rt_controller.py`), ported as
plain Python and wrapped in a thin ROS 2 node.

## Modules

| Module | Ported from | Content |
|---|---|---|
| `law.py` | `rt_law_v3.py` | `LawCfg`, `Law`: source selection ELP / bore1 / model, gated integral, plans A / A_b1 / B / Bp / Bpp, G4, target schedule, leg rules. Pure: no ROS, no I/O. |
| `controller.py` | `rt_controller.py`, `run_cl.speed_rules` | `CtrlCfg`, `PrintWindow`, `Controller` (update every t_u, feed-forward OvCmd, heartbeat, speed check), speed floor and the start-up refusals |
| `rowstore.py` | `bore1/bore1_rt.py` | per-source row store and window estimate |
| `history.py` | `rsi_ring.py` | RSI state history (v_eff, deduped track) |
| `pattern.py` | `rt_readout.Pattern`, `rt_law_v3` helpers | pattern geometry, program speeds, plateaus |
| `params.py` | `run_cl.py` arguments | node parameters with the legacy defaults; config building |
| `replay.py` | – | offline replay of a logged session |
| `node.py` | – | `/krc/state` + `/width/elp` + `/width/bore1` → `/krc/override_cmd`; `/controller/ctrl_record` (`CtrlRecord`) after every update |
| `record.py` | `run_cl.py` `log_update`, `CTRL_COLS` | controller → one `ctrl_<ses>.csv` row (unrounded) |

## Node

```bash
ros2 run cl_controller controller_node --ros-args --params-file config/e1.yaml
```

- Publishes `OverrideCmd` every `step_period_s` (0.05 s) while the controller heartbeat is fresh: the last update
  succeeded within `wd_s` and the newest `KrcState` is younger than `ring_fresh_s`. Otherwise it publishes nothing, so
  `seq` stops and the bridge's watchdog ramps to `ov_default`.
- `source` is the law state (`elp`, `bore1`, `model`) under control, `default` before the first update inside the print
  window, `forced` after a speed-check latch.
- The value is clamped to `[ov_min, ov_max]`, with `ov_min` raised to the speed floor. The bridge clamps again and
  owns `ov_outside`, the watchdog ramp and the reply.
- One `Width` message is one row: `s_tip = s_mm + d_mm`. Messages with `valid = false` are ignored.

## Parameters

Every `run_cl.py` argument that configures the controller is a parameter with the same name (`-` → `_`) and the same
effective default. Where `run_cl.py` uses `None` to mean "derived" or "not given", the parameter uses a sentinel:
`ov_base` 0 (derived), `w12_t` 0, `limit_probe` -1, empty strings.

These arguments belong to other nodes and are not declared here: `pipeline_dir`, `rig`, `wait_for_rsi`, `src_name`,
`cam_index`, `elp`, `elp_instance`, `elp_codec`, `elp_ffmpeg`, `elp_hevc_cq`, `ceiling_light`, `preview`, `bore1_shadow`,
`bore1_dir` and `fusion_params` (see `params.NOT_HERE`).

## Tests

```bash
python3 -m pytest src/cl_controller/test
```

- `test_golden_e1.py`: golden replay of E1, against the legacy code (fixture) and against the online logs.
- `test_law_diff.py`: legacy `rt_law_v3.Law` and the port on the same synthetic sequences.
- `test_legacy_behaviour.py`: speed floor, clamp, leg rules, speed check.
- `test_params.py`: parameter coverage and defaults; `config/e1.yaml` against `session_meta.json`.
- `test_node.py`: node smoke test (needs ROS).

Regenerate the legacy fixture with `python3 src/cl_controller/test/golden/make_e1_fixture.py`.
