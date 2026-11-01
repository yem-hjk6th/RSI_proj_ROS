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
> acting, use `mode:=closed` with plan `Bpp`. The bringup logs this as a warning at start-up, and the session node
> logs it again as a `WARN` line (`startup.SHADOW_WARNING`).

## Bringup (`launch/bringup.launch.py`)

| Argument | Default | |
|---|---|---|
| `mode` | **required, no default** | `shadow` \| `closed` \| `step`. `run_cl.py` has refused a missing `--mode` since 261020 r2: a forgotten mode used to mean 1 % override. |
| `params_file` | **required** | One YAML with a section per node: `krc_rsi_bridge`, `cl_controller`, `width_readout_elp`, `print_session`. `config/e1.yaml` is the example. |
| `image_topic` | `/elp/image_raw` | ELP image topic for the readout. Empty means no readout, which only `step` allows. |
| `start_camera` | `false` | Also start `usb_cam`, publishing on `image_topic`. |
| `camera_params` | – | `usb_cam` parameter YAML (example: `config/elp_usb_cam.yaml`). |
| `out_dir` | `sessions` | The session is written to `<out_dir>/<session>/`: `session_meta.json` and `bag/`. |
| `session` | `YYYYmmdd_HHMMSS` | Session name. |

Nodes started:
- `krc_rsi_bridge`
- `cl_controller`
- `width_readout_elp`
- the recorder (`ros2 bag record`)
- `print_session` (the session node)
- `usb_cam`, when asked.

The start-up checks (`print_session/startup.py`) run in the launch before any node starts. A refusal stops the launch with its reason. The session node runs them again and writes `session_meta.json` before RSI connects.

The session ends 2 s (`print_session.disconnect_timeout_s`) after the last RSI packet (`run_cl.py:882-884`), or on Ctrl+C. The launch then stops every node and the recorder, which closes the bag.

**The bridge's limits come from the controller.** The bringup passes these bridge parameters from the `cl_controller` section and the mode:
- `mode`, `pattern_file`;
- `ov_min` (outer), `ov_max`, `ov_outside`, `ov_default` (derived);
- `v_min_mm_s` (0 in step mode), `fallback_ramp_s`, `wd_s`;
- the window tolerances, `step_seq`, `step_hold_s`, `tau_v`.

In `run_cl.py` both sides shared one `CtrlCfg`. A different value in the `krc_rsi_bridge` section is refused. The bridge's effective window, speed floor included, must equal the controller's.

## Camera

The camera driver is not in this repository.

**Choice: `usb_cam`**, `ros-jazzy-usb-cam` 0.8.1-1. Its Jazzy release is in `ros/rosdistro` `jazzy/distribution.yaml` (release repository `ros2-gbp/usb_cam-release`, tag `release/jazzy/usb_cam/0.8.1-1`).

Why `usb_cam` and not `v4l2_camera` (also released for Jazzy, 0.7.3-1):
- the legacy recorder read the ELP as MJPG at 1920×1200;
- `usb_cam` decodes MJPG to `rgb8` in the driver (`pixel_format: mjpeg2rgb`), which is what the readout takes;
- I did not check how `v4l2_camera` handles MJPG.

`usb_cam` is not a package dependency. Only `start_camera:=true` needs it:

```bash
sudo apt install ros-jazzy-usb-cam
```

- **Encoding.** `mjpeg2rgb` publishes `sensor_msgs/Image` with encoding `rgb8`, which `width_readout_elp` reads (`image.py`: `bgr8`, `rgb8`, `bgra8`, `rgba8`).
- **Stamp.** `header.stamp` is the V4L2 buffer timestamp: for `uvcvideo`, `CLOCK_MONOTONIC` at the start of the frame. `usb_cam` shifts it to wall time with an offset it measures once at start (`usb_cam/utils.hpp`: `get_epoch_time_shift_us`, `calc_img_timestamp`).
  - That is the capture time, on the same clock as `KrcState.stamp`.
  - It is not the time the image reached the readout.
  - The readout gates the frame on that time (`use_header_stamp: true`).
  - A wall-clock step after the camera started (NTP) shifts the stamps. Keep the edge PC's clock disciplined before a session.

## Recording

`ros2 bag record -s mcap --regex '^/(krc/(state|override_cmd|reply_record)|controller/ctrl_record|width/.+)$'` records:
- `/krc/state`, `/krc/override_cmd`;
- `/krc/reply_record` and `/controller/ctrl_record` (the two record topics);
- `/width/*`.

Images are not recorded.

**Storage: mcap.**
- It is the Jazzy default.
- It is one append-only file written in chunks. A crash loses at most the open chunk, and no database journal has to be recovered.
- A session writes several hundred messages/s: `KrcState` and `ReplyRecord` at 250 Hz each, plus the `Width` rows of every ELP frame. Neither format needs tuning for that.
- It can be read without ROS (mcap tools, Foxglove).

## Exporter (`bag_to_legacy`)

It writes `roboter/<ses>/{rsi_data,ov_cmd,ctrl,readout,readout_rows}_<ses>.csv` and copies `session_meta.json` to `raw_video/<ses>/`. Column names and order come from:
- the E1 files (`rsi_data`, `ov_cmd`);
- `CTRL_COLS`, `READOUT_COLS` and `ROWS_COLS` of `run_cl.py`.

Number formats are those of the legacy writers (`rsi_proc.py`, `run_cl._f`).

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

`eval_v3_print.py` uses only `frame` and `leg` from `readout`, so the renumbering does not change its result.

## Tests

| Test | Acceptance |
|---|---|
| `test_roundtrip.py` | 1: E1 CSVs → bag (`test/e1_bag.py`, test-only writer) → export → every carried column equal to the original string. |
| `test_eval_unchanged.py` | 2: the legacy eval on the original logs reproduces the stored eval JSON (keys that need inputs not in this repository are listed). On the exported logs it gives the same result. |
| `test_bringup.py` | 3: shadow bringup against `krc_sim` on loopback for 10 s (no OvCmd in the replies, bag, `session_meta.json`, exporter, the shadow-mode warning in the log). 4: the launch refuses without `mode`. |
| `test_startup.py` | `session_meta` realtime block equal to E1's; speed block equal to the legacy `speed_rules`; the refusals; the shadow-mode warning (shadow only). |
| `test_export_unit.py` | Exporter formats and joins on hand-made messages. |
