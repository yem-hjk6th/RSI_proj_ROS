# print_session

One launch command starts a print session on the edge PC, the way `legacy/realtime_cl_261023/run_cl.py` did. The session is recorded with rosbag2. `bag_to_legacy` turns the bag into the legacy CSV logs, so that `legacy/realtime_cl_261023/eval_v3_print.py` runs unchanged on ROS sessions.

```bash
ros2 launch print_session bringup.launch.py mode:=closed params_file:=config/e1.yaml
```

```bash
ros2 run print_session bag_to_legacy sessions/<ses>/bag --out exported
```

```bash
python3 legacy/realtime_cl_261023/eval_v3_print.py --data exported --session <ses> --pattern <pattern json>
```

> **Shadow mode on `RSI_OV4` means 1 % override.** In shadow mode the reply carries no `<OvCmd>`. On a KRC program that
> uses `RSI_OV4` (`RSI_CREATE("RSI_OV4.rsi")`) the OvCmd input is then 0, `LIMIT1` (1..100) raises it to 1, and the
> robot runs at 1 % override (`legacy/README.md`; `legacy/realtime_cl_261023/run_cl.py` lines 4 and 7). The lab's
> print programs load `RSI_OV4`, so do not use shadow mode there; for a run that computes and logs everything without
> acting, use `mode:=closed` with plan `Bpp`. The bringup refuses shadow mode unless `print_session.krc_rsi_config` is
> `RSI_MIN` (default `RSI_OV4`). With `RSI_MIN` it logs this as a warning at start-up, and the session node logs it
> again as a `WARN` line (`startup.SHADOW_WARNING`).

## Bringup (`launch/bringup.launch.py`)

| Argument | Default | |
|---|---|---|
| `mode` | **required, no default** | `shadow` \| `closed` \| `step`. `run_cl.py` has refused a missing `--mode` since 261020 r2: a forgotten mode used to mean 1 % override. |
| `params_file` | **required** | One YAML with a section per node: `krc_rsi_bridge`, `cl_controller`, `width_readout_elp`, `print_session`, and `width_readout_bore1` when the bore1 readout runs. `config/e1.yaml` is the example. |
| `image_topic` | `/elp/image_raw` | ELP image topic for the readout. |
| `readout` | `true` | `false`: no ELP readout, which only `step` allows. (`image_topic:=` with an empty value is rejected by `ros2 launch` as a malformed argument, so it cannot switch the readout off.) |
| `cameras` | – | `camera_capture` parameter YAML, one section per camera node (examples: `camera_capture/config/cameras_e1.yaml`, `cameras_cage.yaml`). See Camera. |
| `preview` | `false` | Also open the 2×2 preview window of the cameras' preview topics (`camera_capture` `preview_node`). |
| `start_camera` | `false` | Instead of `cameras`: start `usb_cam`, publishing on `image_topic`, without recording. |
| `camera_params` | – | `usb_cam` parameter YAML (example: `config/elp_usb_cam.yaml`). |
| `out_dir` | `sessions` | The session is written to `<out_dir>/<session>/`: `session_meta.json`, `bag/` and, with `cameras`, `raw_video/`. |
| `session` | `YYYYmmdd_HHMMSS` | Session name. |

Nodes started:
- `krc_rsi_bridge`
- `cl_controller`
- `width_readout_elp`
- `width_readout_bore1`, when `print_session.bore1_readout` is true (see bore1 readout);
- the recorder (`ros2 bag record`)
- `print_session` (the session node)
- one `camera_capture` node per section of the `cameras` file, or `usb_cam`, when asked;
- the preview window, when asked.

The start-up checks (`print_session/startup.py`) run in the launch before any node starts. A refusal stops the launch with its reason. The session node runs them again and writes `session_meta.json` before RSI connects.

The session ends 2 s (`print_session.disconnect_timeout_s`) after the last RSI packet (`run_cl.py:882-884`), or on Ctrl+C. The launch then stops every node and the recorder, which closes the bag.

**The bridge's limits come from the controller.** The bringup passes these bridge parameters from the `cl_controller` section and the mode:
- `mode`, `pattern_file`;
- `ov_min` (outer), `ov_max`, `ov_outside`, `ov_default` (derived);
- `v_min_mm_s` (0 in step mode), `fallback_ramp_s`, `wd_s`;
- the window tolerances, `step_seq`, `step_hold_s`, `tau_v`.

In `run_cl.py` both sides shared one `CtrlCfg`. A different value in the `krc_rsi_bridge` section is refused. The bridge's effective window, speed floor included, must equal the controller's.

## bore1 readout

`print_session.bore1_readout: true` starts `width_readout_bore1` (task T11) with the section `width_readout_bore1` of
the parameter file; the bringup sets its `pattern_json` (the controller's pattern) and `session_meta`. Plan `A_b1`
(bore1 → ELP → model, E1's plan) and a `force_seg` with bore1 are refused without it (`run_cl.py:692-695`).

```yaml
width_readout_bore1:
  ros__parameters:
    head: torch
    weight_dir: <BORE1_WEIGHT_DIR>       # cnxt_t_deploy_s0.pt .. s2.pt; not in this repository
    image_topic: /bore1/image_raw
    frame_time: receive                  # or header + header_offset_s, see width_readout_bore1/README.md
```

Refused before any node starts (`startup.bore1_check`): `head` other than `torch` / `replay`; `torch` without a
`weight_dir` that holds the three seed files (the node checks their sha256 before it publishes anything); `replay`
without a `replay_csv`; another `pattern_json` than the controller's; `lockstep`; and, with a `cameras` file, an
`image_topic` that no camera publishes. The legacy `run_cl.py` loaded the head before RSI connected. Here the node loads
it while the other nodes start (sha256 of 333 MB, three models, CUDA graph, warm-up: a few seconds; it logs
`head torch ready in ... s`). Start the program on the KRC after that line. If the node ends while the session runs
and the plan uses bore1, the bringup ends the session; for other plans (e.g. `A`, where bore1 is the fallback) the
session goes on with ELP → model.

The node writes `bore1_readout` into `session_meta.json` (head, frame time, frames, gate counts, predicted, pushed,
errors, head time mean / p95 / max). Its records go into the bag (`/width/bore1/record`), and `bag_to_legacy` writes
them as `raw_video/<ses>/bore1_rt_<ses>.csv`.

## Camera

**`cameras:=<file>` (`camera_capture`, task T10).** One node per section of the file; the section name is the node
name (`elp`, `bore1`, ...). Each node captures its camera, records it in the same process and publishes
`/<name>/image_raw` (capture-time stamps) and `/<name>/preview` (5 Hz, 640 px). The bringup sets for every camera:
- `session` (the file names carry it);
- `video_dir` = `<out_dir>/<session>/raw_video/` (segment files, frame CSVs, `elp_recorder_<ses>.json`);
- `session_meta` = `<session>/session_meta.json`: each camera writes its `cameras.<name>` block (requested,
  read back, measured: frames, dropped, fps, segments, encoder, fallback).

Refused (`print_session/cameras.py`): `cameras` together with `start_camera:=true`; a camera section that sets
`session`, `video_dir` or `session_meta`; two cameras with the same topic or file names; with the readout on, an
`image_topic` that no camera publishes. The example files record from the first RSI packet (`wait_for_rsi`, the legacy
default). Details, formats and the hardware checks: `src/camera_capture/README.md`.

```bash
ros2 launch print_session bringup.launch.py mode:=closed params_file:=config/e1.yaml \
    cameras:=src/camera_capture/config/cameras_e1.yaml preview:=true
```

**Option: `usb_cam`** (`start_camera:=true`), publishing on `image_topic`, without recording. It is not in this
repository.

Before T10 the camera driver was `usb_cam`, `ros-jazzy-usb-cam` 0.8.1-1. Its Jazzy release is in `ros/rosdistro` `jazzy/distribution.yaml` (release repository `ros2-gbp/usb_cam-release`, tag `release/jazzy/usb_cam/0.8.1-1`).

Why `usb_cam` and not `v4l2_camera` (also released for Jazzy, 0.7.3-1):
- the legacy recorder read the ELP as MJPG at 1920×1200;
- `usb_cam` decodes MJPG to `rgb8` in the driver (`pixel_format: mjpeg2rgb`), which is what the readout takes;
- I did not check how `v4l2_camera` handles MJPG.

`usb_cam` is not a package dependency. Only `start_camera:=true` needs it:

```bash
sudo apt install ros-jazzy-usb-cam
```

- **Encoding.** `mjpeg2rgb` publishes `sensor_msgs/Image` with encoding `rgb8`, which `width_readout_elp` reads (`image.py`: `bgr8`, `rgb8`, `bgra8`, `rgba8`).
- **Stamp** (`camera_capture` stamps its images the same way). `header.stamp` is the V4L2 buffer timestamp: for `uvcvideo`, `CLOCK_MONOTONIC` at the start of the frame. `usb_cam` shifts it to wall time with an offset it measures once at start (`usb_cam/utils.hpp`: `get_epoch_time_shift_us`, `calc_img_timestamp`).
  - That is the capture time, on the same clock as `KrcState.stamp`.
  - It is not the time the image reached the readout.
  - The readout gates the frame on that time (`use_header_stamp: true`).
  - A wall-clock step after the camera started (NTP) shifts the stamps. Keep the edge PC's clock disciplined before a session.

## Recording

With `cameras`, the video and the frame CSVs are written by the camera nodes to `<session>/raw_video/` in the legacy
formats (`camera_capture/README.md`, Recording). The bag holds no images.

`ros2 bag record -s mcap --regex '^/(krc/(state|override_cmd|reply_record)|controller/ctrl_record|width/.+)$'` records:
- `/krc/state`, `/krc/override_cmd`;
- `/krc/reply_record` and `/controller/ctrl_record` (the two record topics);
- `/width/*`: `/width/elp`, `/width/bore1` and `/width/bore1/record` (`Bore1Record`, one per processed bore1 frame).

Images are not recorded.

**Storage: mcap.**
- It is the Jazzy default.
- It is one append-only file written in chunks. A crash loses at most the open chunk, and no database journal has to be recovered.
- A session writes several hundred messages/s: `KrcState` and `ReplyRecord` at 250 Hz each, plus the `Width` rows of every ELP frame. Neither format needs tuning for that.
- It can be read without ROS (mcap tools, Foxglove).

## Exporter (`bag_to_legacy`)

It writes `roboter/<ses>/{rsi_data,ov_cmd,ctrl,readout,readout_rows}_<ses>.csv`, `raw_video/<ses>/bore1_rt_<ses>.csv` when the bag has bore1 records, and copies `session_meta.json` to `raw_video/<ses>/`. Column names and order come from:
- the E1 files (`rsi_data`, `ov_cmd`);
- `CTRL_COLS`, `READOUT_COLS` and `ROWS_COLS` of `run_cl.py`;
- `B1_COLS` of `bore1/bore1_rt.py`.

Number formats are those of the legacy writers (`rsi_proc.py`, `run_cl._f`, `Bore1Thread`).

**Columns the topics cannot carry** (`legacy_cols.NOT_CARRIED`; the round-trip test checks that this list is complete):

| File | Column | Export |
|---|---|---|
| `rsi_data` | `pc_ts_monotonic` | written as `pc_ts_perf` (on Linux both are `CLOCK_MONOTONIC`) |
| `rsi_data` | `rsol_x_mm` … `rsol_c_deg` | empty (RSol is parsed by the bridge but not in its packet ring) |
| `readout` | `frame` | `Width` has no frame index: frames are numbered 1, 2, … in stamp order, and `readout_rows` uses the same numbers |
| `readout` | `t_frame_perf` | derived: frame stamp minus the bag's median `KrcState.stamp − ReplyRecord.pc_ts_perf` |
| `readout` | `t_done_perf`, `compute_ms`, `skipped_total`, `n_rows` | empty |
| `readout` | (rows) | frames the readout gated out publish nothing, so they are not in the export |
| `readout` | `w_med_used` | recomputed from the published rows |
| `readout_rows` | `frame` | renumbered as in `readout` |
| `bore1_rt` | `t_frame` | derived: `Bore1Record.stamp` minus the same offset as `readout.t_frame_perf` |
| `bore1_rt` | `frame` | carried, but in a ROS session it is the node's count of processed frames, not the camera's frame index |
| `bore1_rt` | (rows) | frames the node skipped while busy have no record, as in the legacy log |

`eval_v3_print.py` uses only `frame` and `leg` from `readout`, so the renumbering does not change its result.

## Tests

| Test | Acceptance |
|---|---|
| `test_roundtrip.py` | 1: E1 CSVs → bag (`test/e1_bag.py`, test-only writer) → export → every carried column equal to the original string. T11 acceptance 4: the same for `bore1_rt` (only `t_frame` differs: derived). |
| `test_eval_unchanged.py` | 2: the legacy eval on the original logs reproduces the stored eval JSON (keys that need inputs not in this repository are listed). On the exported logs, `bore1_rt` included, it gives the same result. |
| `test_bringup.py` | 3: shadow bringup against `krc_sim` on loopback for 10 s (no OvCmd in the replies, bag, `session_meta.json`, exporter, the shadow-mode warning in the log). 4: the launch refuses without `mode`. T10: `readout:=false` in step mode with a `cameras` file of two synthetic cameras against `krc_sim` (no readout node; `raw_video/` CSVs 1:1 with the files, recording from the first RSI packet, the `cameras` blocks); the refusals of `readout:=false` outside step mode, of a non-boolean `readout` and of `cameras` with `start_camera`. T11: closed mode with plan `A_b1` and the bore1 readout (replay head on E1's log) against `krc_sim` with two synthetic cameras: records in the bag, `bore1_rt` exported, the node's `session_meta` block. |
| `test_cameras.py` | The `cameras` file plan and its refusals; the example camera files keep the legacy profiles. |
| `test_startup.py` | `session_meta` realtime block equal to E1's; speed block equal to the legacy `speed_rules`; the refusals; the shadow-mode warning (shadow only); the bore1 readout plan and its refusals. |
| `test_export_unit.py` | Exporter formats and joins on hand-made messages. |

The tests run on a ROS domain of their own (1–19, set in `test/conftest.py`; `PRINT_SESSION_ROS_DOMAIN_ID` overrides
it): the bringup tests publish `/krc/state` through the bridge, which other packages' node tests running in parallel
react to.

## Cage 2 (T15)

`print_session.cage2: true` (example `config/cage2_bduo.yaml`, the flags of session bduo_20261103_195120) also starts:

- `width_readout_elp_b` (kind B, section `width_readout_elp_b`) on `/elp_b/image_raw` -> `/width/elp_b`, whenever the
  pattern has +X lines (with `elp_b off` it is logged only, as legacy);
- `width_readout_bore2` (camera bore2, section `width_readout_bore2`) when `cl_controller.bore2` is not off. With
  `fallback` / `primary` the session ends if that node ends (as for bore1 with plan A_b1).

Rigs are bound to the role: ELP-A uses the section's `rig_json` or the packaged cage-2 ELP-A rig, ELP-B the packaged
ELP-B rig; a rig of the other role, a rig whose role cannot be determined (`width_readout_elp.config.rig_role`: a
`"role"` key, the packaged role folder or a copy of a packaged rig; never the file name), or the same file for both, is
refused. `cl_controller.bore_sd_max` and
`bore1_offset_mm` / `bore2_offset_mm` are handed to the bore nodes (`sd_max`, `w_offset`); a bore section that sets
another value is refused. The cage-2 refusals of run_cl.py are listed in `print_session/startup.py`. The camera file must
publish the images the readouts read: `camera_capture/config/cameras_cage.yaml` keeps `publish_raw: false` for elp_b
and bore2, which the bringup refuses for a cage-2 session (set it true in the lab copy).

Recorded: `/controller/switch_record` and `/width/elp_b`, `/width/bore2*`; `bag_to_legacy` writes `switch_<ses>.csv`,
`readoutB_<ses>.csv`, `readoutB_rows_<ses>.csv` and `raw_video/bore2_rt_<ses>.csv`, and the ctrl csv has the cage-2
columns.

Patterns with `"geometry"` (fan / semicircle) are refused (through the controller's configuration) until their
controller exists (T16).
