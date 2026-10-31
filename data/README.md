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

Licence: MIT, same as the repository.
