# Data

`e1_20261022_165128/` is one closed-loop print (session E1), recorded on 2026-10-22 with the pipeline in `legacy/realtime_cl_261023/`.

- R1v30 pattern: ten −X legs.
- 120 motor rpm (10:1 gearbox, so the screw turns at about 12 rpm).
- ELP-led control law v3.

The CSV files are gzip-compressed. Read them with `pandas.read_csv(path)`.

| File | Content |
|---|---|
| `roboter/rsi_data_*.csv.gz` | Every RSI packet as received: IPOC, RIst pose, speed, override, rpm |
| `roboter/ov_cmd_*.csv.gz` | Override sent in each reply, with its source |
| `roboter/ctrl_*.csv.gz` | Controller updates (law state per update) |
| `roboter/readout_*.csv.gz`, `readout_rows_*.csv.gz` | ELP width readout, per frame and per row (d = distance behind the nozzle, s_dep = arc length, w = width in mm) |
| `roboter/eval_v3_*.json/.md`, `print_card_*.json` | Run evaluation and run notes |
| `raw_video/elp_frames_*.csv.gz` | ELP frame index: frame_idx, timestamps, video segment |
| `raw_video/bore1_rt_*.csv.gz`, `borescope_frames_*.csv.gz`, `fusion_*.csv.gz` | Borescope readout, borescope frame index, source fusion |
| `raw_video/session_meta.json` | Session metadata |
| `elp_frames/frame_NNNNNN.jpg` | 40 ELP frames (every 5th frame of video segment 3 that has readout rows); `NNNNNN` = `frame_idx` |

**About the frames.** They come from an MJPEG re-encode at quality 95, not from the original stream. A legacy readout run on these frames will therefore differ slightly from the online `readout_rows`. For golden tests, compare the port with the legacy code on the same frames.

## Sessions of 2026-11-03 (cage 2: ELP-A, ELP-B, bore1, bore2)

Recorded with the lab copy in `legacy/realtime_cl_cage2_semi_261103/` (tasks T13, T14). Logs only: no video and no
frames. Same file layout as E1, plus the second camera of each pair.

| Folder | Print | Pattern / program | Run |
|---|---|---|---|
| `fan1cl_20261103_222754/` | FAN1 closed loop, bore-led | `pattern_usr_ELP_FAN1_261103.json` / `usr_ELP_FAN1_OL_261103.src`: a 155 mm lead and a 155 mm top line at 0 deg, fourteen 60–66 mm lines at −45 … +45 deg | `--mode closed --plan A_b1 --control-sources elp,bore1 --bore2 primary`, `--fan-ctrl-max-deg 90 --fan-bore-max-deg 90`, OvCmd 40 outside the print window |
| `arc2ol_20261103_235936/` | ARC2 open loop | `pattern_usr_ELP_ARC2_261103.json` / `usr_ELP_ARC2_OL_261103.src`: a straight lead, four rows of semicircles (R 19.375 and R 12.917, every lobe turning the tangent 180 deg), a straight end row | `--mode closed --plan Bpp` (OvCmd 40 everywhere), ELP-A / ELP-B readouts and bore1 / bore2 (shadow) inference logged live, caps 90 / 90 |

The `run` column is from `raw_video/session_meta.json` (`realtime`, `bore2`). Both use 120 motor rpm after a purge
at 400 rpm (program header).

| File | Content |
|---|---|
| `roboter/readoutB_*.csv.gz`, `readoutB_rows_*.csv.gz` | ELP-B readout, as `readout_*` for ELP-A |
| `roboter/switch_*.csv.gz` | Source switches of the controller |
| `roboter/semi_gate_*.csv.gz` (ARC2) | Gate log: one row per gated ELP or bore frame with the geometry at the tip and at the observed position (`legacy/realtime_cl_cage2_semi_261103/README.md`) |
| `roboter/failuremap_summary_*_lab.csv` (ARC2) | The lab's failure-map summary, kept as a reference. Its ELP rows are binned by the angle at the tip, not where the ELP looks (known issue 1 of that README) |
| `raw_video/bore2_rt_*.csv.gz`, `borescope_frames_*_bore2.csv.gz` | bore2 readout and frame index, as bore1 |
| `raw_video/elpB_frames_*.csv.gz` | ELP-B frame index |

- `bore1_rt` / `bore2_rt` list every frame the bore thread processed with its gate decision (`gate` = `ok` or the
  reason). `readout_*` lists every frame the ELP reader processed; `leg` is empty when the gate rejected it. Frames the
  reader skipped while busy are counted in `skipped_total` and have no row.
- Time base: `t_frame` / `t_frame_perf` and the RSI `pc_ts_perf` are on the same performance-counter clock.

Licence: MIT, same as the repository.
