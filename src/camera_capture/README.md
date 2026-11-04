# camera_capture

Capture of the cell's cameras in ROS 2, with the recording in the same process (task T10). It ports the legacy
capture and recorders:

- `legacy/pipeline_cage1012/elp.py` (`ElpCam`): MJPG capture, controls with read-back, MJPG AVI writer, frame CSV;
- `legacy/realtime_cl_261023/elp_hevc.py`: the ELP recorder of 2026-10-23, HEVC through `ffmpeg` with the MJPG AVI
  fallback;
- `legacy/pipeline_cage1012/borescope.py` (`BoreScope`): 1280×720, mp4v, never sets exposure;
- `legacy/pipeline_cage1012/config.py`: the camera profiles and segment lengths.

**Nothing here has run with a camera or a GPU.** CI runs the core with synthetic frames, an `ffmpeg` stub and the real
`ffmpeg` with a software encoder in place of `hevc_nvenc`. The hardware checks are listed below.

The core has no ROS imports. `node.py` and `preview_node.py` are thin ROS wrappers.

## Nodes

### `camera_node` (one per camera)

```bash
ros2 run camera_capture camera_node --ros-args -r __node:=elp --params-file src/camera_capture/config/cameras_e1.yaml
```

Usually started by the bringup (`cameras:=<file>`, see `print_session/README.md`). The node name is the camera name.

Start-up, as `run_cl.py` / `ElpCam`:
1. the encoder self test (`codec: hevc`) before the camera opens;
2. open the camera by a stable device path, set the format and the V4L2 controls, read everything back and log
   requested and read-back values;
3. write the camera's block of `session_meta.json` (`cameras.<name>`);
4. capture; record at once, or from the first `/krc/state` message (`wait_for_rsi: true`, the legacy default).

| Topic | Type | |
|---|---|---|
| `/<name>/image_raw` | `sensor_msgs/Image` `bgr8`, best effort, depth 1 | Every frame while the publisher keeps up. `header.stamp` = capture time on the clock of `KrcState.stamp`. Built only while someone subscribes. |
| `/<name>/preview` | `sensor_msgs/Image` `bgr8`, reliable, depth 1 | Decimated to `preview_hz` (5 Hz) by capture time: the first frame of each period, from the first period boundary after the start. Downscaled to `preview_width` (640). |

**Threads.** The capture thread reads, stamps and hands each frame to the recorder queue without waiting (a full
queue drops the frame and counts it, as `ElpCam`). The writer thread writes the files and the CSV. A publisher thread
takes the newest frame; when it falls behind it skips frames (counted), and the capture and the recording do not
notice. The preview is chosen on the capture thread, which sees every frame (the first frame of each `1 / preview_hz`
period by capture time), and waits in a slot of its own, so a publisher that falls behind still publishes that frame
and not a later one. Nothing in the capture or recording path waits on ROS.

**Stamps.** `KrcState.stamp` is the wall clock (`time.time()`). A V4L2 buffer carries its capture time on
`CLOCK_MONOTONIC` (uvcvideo: start of frame; OpenCV's `CAP_PROP_POS_MSEC` on the V4L2 backend). The stamp is that time
plus `wall − monotonic` measured once at start (as `usb_cam` does), kept strictly increasing. A frame without a usable
buffer time (none, or not within 0.5 s before the read) gets its read time; both counts are in `measured.stamps`. The
frame CSV keeps the legacy receive times (`pc_ts_*` after the read).

### `preview_node`

One window with up to four preview topics tiled 2×2: `topics[0]`, `topics[1]` = left column (pair 1: top, bottom),
`topics[2]`, `topics[3]` = right column (pair 2). It subscribes to the preview topics only and nothing subscribes to
it, so it is off the control path. It also publishes the tiled image on `/cameras/preview`; without a display it only
publishes. Bringup: `preview:=true`.

## Recording

| Codec | Writer | Files | Segment | Legacy |
|---|---|---|---|---|
| `hevc` | `ffmpeg` subprocess, raw `bgr24` on stdin: `hevc_nvenc -preset p4 -tune hq -rc vbr -cq 18 -b:v 0`, yuv420p full range BT.601, `bt470bg` / `smpte170m` tags, fragmented MP4 (250 ms fragments) | `elp_<ses><tag>_<seg:03d>_<start>.mp4` | 60 s | `elp_hevc.py` (command unchanged; a test compares it with the legacy `build_cmd`) |
| `mjpg` | OpenCV's MJPEG encoder into AVI, JPEG quality 95 | `.avi` | 60 s | `ElpCam` (the E1 files) and the HEVC fallback |
| `mp4v` | `cv2.VideoWriter` mp4v | `bore_<ses><tag>_<seg:03d>_<start>.mp4` | 900 s | `BoreScope` |

Frame CSV: `<csv_prefix>_<ses><tag>.csv` (`elp_frames_<ses>.csv`, `borescope_frames_<ses>.csv`), columns
`pc_ts_monotonic,pc_ts_wall,frame_idx,segment_idx,frame_in_segment,video_file,pc_ts_perf`, CRLF line ends.
`tag` is `""` for one camera of a kind (the E1 names), else e.g. `_cam1` (legacy `pipeline.cam_tags`) or `_b`.

**Segments.** A segment is opened by its first frame and named with its start: the session's first frame, then the
frame that ended the previous segment. It ends after the first frame whose `pc_ts_monotonic` is `segment_s` or more
past that start. The legacy writer measured this on its own clock (Windows, 15.6 ms ticks); on E1 the port gives the
same CSV for the borescope (byte for byte) and for the ELP's first three segments, and segments 4–7 end 1 or 2
frames earlier; all file names are the same (`test_golden_e1.py`).

**CSV rows 1:1 with the frames in the files** (the HEVC recorder's rules, `elp_hevc.ElpCamHEVC`):
- `mjpg` / `mp4v`: the frame is written, then its row.
- `hevc`: a row is committed `ring` (48) frames after its frame went into the pipe; at a segment end all at once.
- **Fallback** to the MJPG AVI writer, once, for the rest of the session: no `ffmpeg` or a failed self test (before
  the session); `ffmpeg` cannot start a segment; a write error (ffmpeg exited); a write blocked longer than `block_s`
  (2 s, the watchdog kills ffmpeg); a capture queue of `backlog` (128) frames or more. The frames of the ring that are
  not in the HEVC file (counted with `fmp4_frames`) go into the AVI with their rows, so rows and frames stay 1:1.
  Rows already committed for frames that never reached the file are counted as `lost_video_frames`.
- Added to the legacy rules: if `ffmpeg` has exited by a segment end or the session end, the same fallback runs
  instead of committing the ring blindly.
- Every fallback is logged (`RECORDER FALLBACK ...`) and listed in `session_meta.json` (`measured.fallback`) and in
  the sidecar `elp_recorder_<ses>.json` (as the legacy one).

`ffmpeg` is the `ffmpeg` parameter if given (the only candidate then), else `$RT_FFMPEG`, else `ffmpeg` on `PATH`. It
runs in its own session, so a Ctrl+C in the terminal does not reach it (stdin EOF ends it; legacy: no console).

## Parameters

Examples: `config/cameras_e1.yaml` (ELP + bore1, as E1), `config/cameras_cage.yaml` (two ELPs, two borescopes).
Device paths in them are placeholders.

| Parameter | Default | |
|---|---|---|
| `kind` | `elp` | `elp`, `borescope` (refuses any exposure control: legacy `borescope.py`), `generic` |
| `device` | – | `/dev/v4l/by-id/...` or `/dev/v4l/by-path/...`; a bare `/dev/videoN` is refused (`require_stable_path`) |
| `fourcc`, `width`, `height`, `fps` | `MJPG`, 1920, 1200, 60.0 | requested format, read back and logged; `fourcc: ""` = the driver's default. The open refuses a wrong frame size or less than `min_fps_ratio` (0.5) of `fps` (`ElpCam`). |
| `controls` | none | V4L2 controls in `v4l2-ctl` names and units, set in this order, every one read back: `[auto_exposure=1, exposure_time_absolute=78, gain=0, white_balance_automatic=0, white_balance_temperature=4600]` for the ELP. Omit for none. |
| `codec` | `hevc` | `hevc`, `mjpg`, `mp4v` |
| `segment_s`, `fallback_segment_s` | 60, 60 | segment length of the codec and of the MJPG AVI fallback |
| `file_prefix`, `csv_prefix`, `tag` | `elp`, `elp_frames`, `""` | file names |
| `cq`, `fps_tag`, `jpeg_quality`, `ffmpeg` | 18, 78.4, 95, `""` | encoder settings (legacy) |
| `queue_frames` | 256 | capture → writer queue (6.9 MB per ELP frame) |
| `wait_for_rsi` | false | record from the first message on `state_topic` (`/krc/state`); the examples set true |
| `session`, `video_dir`, `session_meta` | – | set by the bringup: session name, `<session>/raw_video`, `<session>/session_meta.json` |
| `image_topic`, `preview_topic`, `frame_id` | `/<name>/image_raw`, `/<name>/preview`, `<name>` | |
| `publish_raw` | true | false = no raw image topic (a camera nothing reads) |
| `preview_hz`, `preview_width` | 5.0, 640 | |
| `source` | `v4l2` | `synthetic` (tests, demos) or `file` (`device` = a video file) |

**ELP exposure in V4L2 units.** E1 ran DirectShow `CAP_PROP_EXPOSURE -7` = 2^-7 s = 7.8125 ms. `exposure_time_absolute`
is in 100 µs, so 78 (7.8 ms). This is computed, not measured: hardware check 3 compares a frame with a legacy frame.
`auto_exposure` 1 is V4L2's manual mode; it must come before the exposure, which is inactive while auto exposure is on
(the same for the white balance).

**Two ELPs.** Both units report the same name and USB serial (`legacy/pipeline_cage1012/elp.py`), so their
`/dev/v4l/by-id` links collide. Use `/dev/v4l/by-path` links (the USB port) for the ELPs, and check the mapping after
every re-plug or `usbipd attach`. A UVC camera has two nodes; the capture node is `...-video-index0`.

## session_meta.json

Each camera writes `cameras.<name>` at open and again at stop (a locked read-modify-write shared with the session
node, `session_meta.py`):

```
cameras.elp: name, kind, device, resolved, backend, stored ("raw sensor orientation"), frames_csv, file_prefix, tag,
             requested {fourcc, width, height, fps, buffers, controls}, readback {same}, control_errors,
             control_mismatch, controls_available (v4l2-ctl --list-ctrls), recorder_choice (self test), final,
             measured {frames_read, frames_written, frames_dropped_queue, read_fails, segments, duration_s,
                       fps_measured, fps_at_open, frames_captured, encoder {requested, at_start, at_end}, fallback,
                       lost_video_frames, stamps, published {raw, preview, skipped}, recorder {...}}
```

The legacy `elp` block had `requested` controls, `readback` and `measured` (`frames_read`, `frames_written`,
`frames_dropped_queue`, `read_fails`, `segments`, `duration_s`, `fps_measured`, `fps_at_open`); those keys are kept.

## Tools

```bash
ros2 run camera_capture camera_capture_verify <session>/raw_video
```

Rows of every frame CSV against the frames counted in each file (`fmp4_frames` for MP4, the `00dc` chunks for AVI).

```bash
ros2 run camera_capture camera_capture_verify --mean <new frame> <legacy frame> --roi <x,y,w,h> --tol-pct 5
```

Mean grey level of two frames (image files, or `<video>@<n>` for frame `n` of a segment).

```bash
ros2 run camera_capture camera_capture_bench --codec hevc --fps 78.4 --seconds 20
```

Synthetic 1920×1200 frames through the capture thread, queue, writer and encoder: recorded rate, queue drops, pipe
write times, CSV check.

## In-process recording or a separate recorder node

The task asks to choose between recording in the capture process and a separate recorder node that subscribes to the
raw image only with a frame rate measured on the lab laptop. That measurement has not been made (no camera in the
session that wrote this). The node records in-process, as the legacy pipeline did (capture, recording and read-out in
one process, threads): a raw 1920×1200 `bgr8` image is 6.9 MB, about 540 MB/s at 78 fps per subscriber, and a
recorder that subscribes to it would add one more copy and a serialization per frame. Hardware check 2 measures the
in-process design with and without a raw-image subscriber; the decision is to be confirmed with those numbers.

## Tests

| Test | Acceptance |
|---|---|
| `test_recorder.py` | 1: synthetic frames → recorder (HEVC on the ffmpeg stub, and on the real ffmpeg with libx264 in place of hevc_nvenc): segment files and CSV rows 1:1, counted in each fragmented MP4. 3: no ffmpeg, ffmpeg not on `PATH`, no NVENC (stub and the real ffmpeg without a GPU), ffmpeg cannot start a segment: fallback logged, 1:1. 4: ffmpeg killed mid-segment, exits by itself, dies before the session end, a blocked write (watchdog), a full backlog: 1:1, lost frames counted. 6: `test_hevc_nvenc_selftest_and_recording` skips without NVENC. Queue drops counted. |
| `test_golden_e1.py` | 2: the E1 frame times through the port give the E1 CSVs (header, rows, file names). |
| `test_capture.py` | 5: preview decimation (5 Hz for any faster rate), chosen on the capture thread (the period's first frame also when the publisher falls behind), stamps = buffer capture time on the wall clock, strictly increasing, read-time fallback, the synthetic source's buffer times on its schedule (as a V4L2 buffer when the reader is late); recording start, read failures, the `measured` block, stable device paths. |
| `test_node_ros.py` | 5 through ROS: image and preview topics on a synthetic source, stamps, preview rate, recording 1:1, the `session_meta` block, recording from the first RSI packet; a borescope refuses exposure. |
| `test_formats.py` | names and rows; `build_cmd`, `fmp4_frames` and the self-test frames against the legacy module; the AVI and MP4 counters. |
| `test_controls_meta.py` | controls set in order and read back (fake camera, fake `v4l2-ctl`), exposure units, concurrent `session_meta` writers, the verify tool on E1 frames. |

## Hardware checks (lab laptop; not run)

1. **Formats and controls.** For each camera: `v4l2-ctl -d <link> --list-formats-ext` and `--list-ctrls`; write down
   the device-path-to-camera mapping (`ls -l /dev/v4l/by-id /dev/v4l/by-path`), and check it again after a re-plug.
   Expected: ELP MJPG 1920×1200 at 60 fps listed; the control names of `cameras_e1.yaml` exist.
2. **10 minutes at the ELP's native rate** with the recorder (`codec: hevc`): `measured.frames_dropped_queue` 0,
   `camera_capture_verify` PASS (CSV rows = file frames), `encoder.at_end` `hevc` (NVENC in use, also in
   `nvidia-smi`), CPU load (`top`, per process), `fps_measured` (E1: 78.4). Once with nothing subscribed to
   `/elp/image_raw`, once with `width_readout_elp` and once with an extra `ros2 topic hz /elp/image_raw`, to decide
   in-process against a separate recorder node.
3. **Exposure.** One frame of the same scene in the same light as a legacy frame (`data/e1_20261022_165128/elp_frames/`
   or a new Windows capture): `camera_capture_verify --mean <new> <legacy> --roi <bed region> --tol-pct 5`. Mean grey
   level within ±5 %; else adjust `exposure_time_absolute`.
4. **Fallback writer rate.** `camera_capture_bench --codec mjpg --fps 78.4 --seconds 60`: `dropped_queue` 0. The
   in-process MJPG AVI writer must keep up alone when NVENC fails (legacy laptop, Windows: ~336 fps). In the cloud
   session that wrote this (4 vCPU, no GPU) it drained only about 54 fps of synthetic 1920×1200 frames.
