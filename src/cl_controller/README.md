# cl_controller

The v3 closed-loop bead-width control law (`legacy/realtime_cl_261023/rt_law_v3.py` and `rt_controller.py`), ported as
plain Python and wrapped in a thin ROS 2 node. It includes the multi-layer (walls) variant
`legacy/realtime_cl_261023_ml/` for the wall programs `usr_ELP_W4L28_*` (T12, see below).

## Modules

| Module | Ported from | Content |
|---|---|---|
| `law.py` | `rt_law_v3.py` | `LawCfg`, `Law`: source selection ELP / bore1 / model, gated integral, plans A / A_b1 / B / Bp / Bpp, G4, target schedule, leg rules. Pure: no ROS, no I/O. |
| `controller.py` | `rt_controller.py`, `run_cl.speed_rules` | `CtrlCfg`, `PrintWindow`, `Controller` (update every t_u, feed-forward OvCmd, heartbeat, speed check), speed floor and the start-up refusals |
| `rowstore.py` | `bore1/bore1_rt.py` | per-source row store and window estimate |
| `history.py` | `rsi_ring.py` | RSI state history (v_eff, deduped track; `veff_at_s(pass_only)` of the walls variant) |
| `pattern.py` | `rt_readout.Pattern`, `rt_law_v3` helpers | pattern geometry (with the walls' `z_off` layers), program speeds, plateaus |
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
  window, `forced` after a speed-check or wall stop rule latch.
- The value is clamped to `[ov_min, ov_max]`, with `ov_min` raised to the speed floor. The bridge clamps again and
  owns `ov_outside`, the watchdog ramp and the reply.
- One `Width` message is one row. The tip position is `s_tip_mm` when it is finite, else `s_mm + d_mm`. The sum
  can be 1 ulp off the publisher's tip, which matters for a row on the window edge. Messages with `valid = false` are
  ignored.
- `lockstep` (default `false`) is a hook for replay tests (`krc_replay`). With `true` there is no timer and no wall
  clock. Each `std_msgs/Float64MultiArray` on `/cl_controller/step` (`data = [t_update, n_state, n_elp, n_bore1]`)
  applies the first `n_*` queued inputs of each topic, runs one law update at `t_update` and publishes one
  `OverrideCmd` and one `CtrlRecord`. In this mode `Width.stamp` is taken as the time the row store receives the row.

## Multi-layer walls (T12)

Port of the five modules of `legacy/realtime_cl_261023_ml/` (its `README.md`). The three wall patterns are in
`patterns/` (`pattern_usr_ELP_W4L28_{CL,OLg,OLs}_261023.json`, byte-identical to the legacy ones). Everything below is
active only with a pattern whose segs csv has a `z_off` column; single-layer patterns run the unchanged code.

- `Pattern`: every A leg carries its planned offset above the layer-1 plane (`z_off` of its first A segment). The leg /
  on-A lookup takes the nearest planned layer and that layer's leg at the tip's y, so the unseen +X passes and a pass
  between two layers are off the A legs. `plane_dz(leg)` is the leg's offset.
- `PrintWindow`: opens on layer 1 as before; closes at the last print point only at the planned last Z (`last_z` of
  the pattern json): a wall passes its last x / y on every layer.
- `control_sources` (`LawCfg.sources`): the sources that may steer. Walls use `elp`: the bore1 head is a layer-1 model;
  bore1 is still estimated and logged (`valid_b1`), but never takes control. Items are stripped as in `run_cl.py`
  (`"bore1, elp"` = `"bore1,elp"`).
- `elp_b on` (cage 2) is refused with a multi-layer pattern: the cage-2 line lookup (`_where2`) uses the layer-1
  geometry, so above layer 1 every leg would be held.
- `sat_stop_s` (`CtrlCfg.sat_s`, `sat_min_dz` 0.6 mm), the wall stop rule: on a seen leg of layer >= 2 (planned offset
  >= `sat_min_dz`), OvCmd from the feedback (plan not `Bpp`) at `ov_max` for `sat_s` in a row latches forced open
  loop like the speed check (`sat_alarm`; the node then publishes source `forced`, which the bridge latches). Default
  (`-1`) as in `run_cl.py`: 2.0 s on wall patterns, off (0) on single-layer patterns whatever is given; 0 = off.
- `StateHistory.veff_at_s(pass_only)`: ported, but, as in the legacy `_ml` pipeline, no caller sets it
  (`RowStore.estimate` is the unchanged legacy `bore1_rt.RowStore.estimate`). Consequence, measured in the synthetic
  golden run: on the first leg of every layer >= 2 (the -X pass right after the same wall's +X pass) the stale +X
  packets hide the new pass, and the ELP estimate first becomes valid at s_tip 50..80 mm instead of about 30 mm.
- The legacy `ctrl` CSV has no new column (`sat_alarm` went to the session summary only), so `CtrlRecord` is unchanged;
  the latch shows as `forced`. `session_meta.json` (print_session) records `sat_s`, `sat_min_dz` and `sources` in its
  `realtime` block, as the legacy `_ml` `run_cl.py` does.

The RSI bridge has its own print window (it sends `ov_outside` outside it); it has the same `last_z` gate
(`krc_rsi_bridge/window.py`), as the legacy RSI process used the `_ml` `PrintWindow`. `krc_eki_bridge` has no print
window: outside the window the controller itself publishes `ov_default`.

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
- `test_params.py`: parameter coverage and defaults (single-layer and `_ml` `run_cl.py`); `config/e1.yaml` against
  `session_meta.json`.
- `test_node.py`: node smoke tests (need ROS), also with a wall pattern.
- `test_walls_ml.py` (T12): the walls changes against `legacy/realtime_cl_261023_ml` piece by piece: patterns and
  `Pattern`, config fields, the law with `sources`, `veff_at_s(pass_only)`, print window, leg lookup, parameters.
- `test_walls_golden.py` (T12 acceptance test 2): `walls_sim.py` runs the wall program W4L28_CL closed loop on
  `krc_sim`'s kinematics with synthetic widths (ELP and bore1), the readout gate of `width_readout_elp` and this
  controller; it must reproduce the legacy `_ml` modules on the same run (fixture `walls_w4l28_cl_legacy.json.gz`):
  every frame's gate decision, leg and layer offset, the OvCmd of every step and packet, every law update. The plant
  is synthetic; no wall session was logged.
- `test_walls_stop_rule.py` (T12 acceptance test 3): the wall stop rule latches on layer 2, not on layer 1, not on a
  single-layer pattern (even with the rule forced on), not in plan `Bpp`, not with `sat_stop_s` 0, and not when two
  legs each stay below `sat_stop_s`; each case port and legacy `_ml` alike.

Regenerate the legacy fixtures with `python3 src/cl_controller/test/golden/make_e1_fixture.py` and
`python3 src/cl_controller/test/golden/make_walls_fixture.py`.
