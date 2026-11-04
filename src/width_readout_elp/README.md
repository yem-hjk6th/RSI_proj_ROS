# width_readout_elp

ELP chroma-edge bead-width readout, ported from `legacy/realtime_cl_261023/rt_readout.py`, with the multi-layer
(walls) changes of `legacy/realtime_cl_261023_ml/rt_readout.py` (T12).

| Module | Content |
|---|---|
| `readout.py` | `Rig`, `Pattern`, `walk`, `frame_rows` (= `sample_chroma` + `rows_from_chroma`), `RtReadout` with the frame gates. numpy + OpenCV, no ROS, no SciPy. |
| `rsi_ring.py` | Causal, deduplicated RSI track (`push`, `newest_t`, `track_at`) as in the legacy ring. |
| `replay.py`, `cli.py` | Offline run on a frame folder + RSI log; writes the legacy `readout_rows` csv (`frame, d, s_dep, w`). |
| `path_readout.py` | `PathReadout` (T13): the readout of the fan / semicircle patterns, on `path_geometry.gates`. |
| `node.py` | `sensor_msgs/Image` + `/krc/state` -> `krc_interfaces/Width` on `/width/elp`. |

`config/rig/` and `config/pattern/` are byte-identical copies of the legacy rig (CAGE1012, with the occlusion mask)
and the R1v30_CLp pattern; a test checks that they stay identical.

## Multi-layer walls (T12)

With a pattern whose segs csv has a `z_off` column (the wall programs `usr_ELP_W4L28_*`, in `cl_controller/patterns/`;
`print_session` hands the readout the controller's pattern), every A leg carries its planned offset above the layer-1
plane. The gate then takes the nearest planned layer and that layer's leg at the tip's y (the unseen +X passes and a
pass between two layers give `z_or_dir`), and the readout plane is lifted by the leg's offset (`Gate.dz`), so it stays
where it is on layer 1 relative to the tool. `Width.leg` is that layer's leg. Without the column, and for the one-argument
`z_cmd(x)` / `leg_at(y)` that the bore1 gate calls, nothing changes.

## Fan and semicircle patterns (T13)

A pattern json with `"geometry"` `fan` (FAN1) or `semi` (ARC2) switches the node to `PathReadout` (ELP-A on the A legs),
the port of the legacy `FanReadout` / `SemiReadout` (`legacy/realtime_cl_cage2_semi_261103/`); every other pattern runs
`RtReadout` unchanged.

- Gate: `path_geometry.gates.elp_gate` at `t_frame - lat_s`: `stale`, `no_track`, `z_or_dir` (`|z - z_cmd| > 0.5`, or
  motion over 0.1 s along the local tangent below `dir_min_mm`), `leg_or_xwin` (no A line within `line_tol_mm`, or x
  outside `x_window`), `min_s_tip`. No angle cap. `Width.leg` is the line's leg, `s_tip_mm` the arc length along it.
- Grid: rows at `s_tip - d` (d 12-25 mm) along the line or row (through the lobe joints of ARC2), columns along the local
  normal; the profile math of `rows_from_chroma` with the MDF band of the line's `mdf_side`; `s_dep = s_tip - d`.
- Semi patterns with `gate_log_dir` set: one row per gated frame in `<gate_log_dir>/semi_gate_<camera>.csv` (the legacy
  `semi_gate_<ses>.csv` columns, cam `elpA`, `s_obs` = `s_tip` as the legacy ELP rows).
- `pattern_json` may be a file name in `path_geometry/patterns/` (`pattern_usr_ELP_FAN1_261103.json`,
  `pattern_usr_ELP_ARC2_261103.json`). The cage-2 rigs are not packaged (out of scope of T13): give `rig_json`.
- `PathReadout(kind="B")` is the ELP-B readout (`FanReadoutB` / `SemiReadoutB`); there is no ELP-B node yet.

```bash
ros2 launch width_readout_elp width_readout_elp.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json \
  rig_json:=<RIG_JSON>
```

## Offline CLI

```bash
ros2 run width_readout_elp width_readout_elp_offline \
  --frames data/e1_20261022_165128/elp_frames \
  --index data/e1_20261022_165128/raw_video/elp_frames_20261022_165128.csv.gz \
  --rsi data/e1_20261022_165128/roboter/rsi_data_20261022_165128.csv.gz \
  --out readout_rows.csv
```

## Node parameters

`camera` (also the `Width.source`, default `elp`), `image_topic` (`/elp/image_raw`), `state_topic` (`/krc/state`),
`width_topic` (`/width/elp`), `rig_json`, `pattern_json`, `root` (empty = packaged files), `d_min` (12.0),
`use_header_stamp` (true: frame time = image header stamp; false: receive time), `gate_log_dir` (semi patterns: the
gate log folder; empty = no log). Example: `config/width_readout_elp.yaml`; `launch/width_readout_elp.launch.py`
takes the string parameters as launch arguments.

The frame time and the `KrcState.stamp` must be on the same clock (PC time). Supported encodings: bgr8, rgb8,
bgra8, rgba8. `Width.leg` carries the pattern's leg number as in the legacy logs.

## Tests

- `test_golden.py`: the port against the legacy readout on the 40 E1 frames (fixture
  `test/fixtures/golden_legacy_e1.csv`, regenerate with `python3 src/width_readout_elp/test/make_golden.py`): same
  rows, width within 1e-6 mm; and the online check against `readout_rows_*.csv.gz` (reported, not asserted zero).
- `test_gates.py`: synthetic contrast, coverage and walk-limit cases, and the RSI frame gates, port vs legacy.
- `test_walls.py` (T12): `Pattern` and the frame gates on wall tracks (layers, +X passes, OLg / OLs offsets) against
  `legacy/realtime_cl_261023_ml/rt_readout.py`, the readout plane lift on E1 frames (dz = 0 bit-identical to the
  single-layer readout), and the single-layer gate unchanged.
- `test_path_readout.py` (T13): `PathReadout` against the legacy `FanReadout(B)` / `SemiReadout(B)` with the cage-2 rigs:
  the gate on this package's ring = the path gate on the legacy ring = the live decision on every ELP-A / ELP-B frame of
  both 2026-11-03 sessions; `read` on synthetic bead images at gated tips (rows within 1e-9); `make_readout`; the gate log.
- `test_launch_fan.py` (T13): `ros2 launch` with FAN1 by name; Width on a FAN1 line, none when moving backwards.
