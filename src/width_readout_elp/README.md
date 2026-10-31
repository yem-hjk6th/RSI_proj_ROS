# width_readout_elp

ELP chroma-edge bead-width readout, ported from `legacy/realtime_cl_261023/rt_readout.py`.

| Module | Content |
|---|---|
| `readout.py` | `Rig`, `Pattern`, `walk`, `frame_rows` (= `sample_chroma` + `rows_from_chroma`), `RtReadout` with the frame gates. numpy + OpenCV, no ROS, no SciPy. |
| `rsi_ring.py` | Causal, deduplicated RSI track (`push`, `newest_t`, `track_at`) as in the legacy ring. |
| `replay.py`, `cli.py` | Offline run on a frame folder + RSI log; writes the legacy `readout_rows` csv (`frame, d, s_dep, w`). |
| `node.py` | `sensor_msgs/Image` + `/krc/state` -> `krc_interfaces/Width` on `/width/elp`. |

`config/rig/` and `config/pattern/` are byte-identical copies of the legacy rig (CAGE1012, with the occlusion mask)
and the R1v30_CLp pattern; a test checks that they stay identical.

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
`use_header_stamp` (true: frame time = image header stamp; false: receive time). Example: `config/width_readout_elp.yaml`.

The frame time and the `KrcState.stamp` must be on the same clock (PC time). Supported encodings: bgr8, rgb8,
bgra8, rgba8. `Width.leg` carries the pattern's leg number as in the legacy logs.

## Tests

- `test_golden.py`: the port against the legacy readout on the 40 E1 frames (fixture
  `test/fixtures/golden_legacy_e1.csv`, regenerate with `python3 src/width_readout_elp/test/make_golden.py`): same
  rows, width within 1e-6 mm; and the online check against `readout_rows_*.csv.gz` (reported, not asserted zero).
- `test_gates.py`: synthetic contrast, coverage and walk-limit cases, and the RSI frame gates, port vs legacy.
