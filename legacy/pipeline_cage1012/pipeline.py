"""
Borescope + RSI synchronized capture orchestrator.

Default behavior:
  1. Spawn RSI listener thread (UDP 59152, replies zero RKorr)
  2. Open every borescope in config.CAM_INDICES, start continuous mp4 capture
     (15-min chunked) on each -- one recording thread per camera, one RSI listener
  3. Print status every STATUS_PRINT_INTERVAL_S
  4. Stop on Ctrl+C, OR when RSI was once connected and then disconnects

`wait_for_rsi=True` defers borescope recording until first RSI packet
arrives (matches the older init_recorder_svo_csv.py pattern). The
default `wait_for_rsi=False` records globally — the user explicitly
chose this: easier offline trimming, no missed startup frames.

File names (2026-09-15, multi-camera): with ONE index the names are the legacy ones
(`bore_<ts>_001_<seg>.mp4`, `borescope_frames_<ts>.csv`); with several, every camera gets
`_cam<idx>` after the session ts (`bore_<ts>_cam1_001_<seg>.mp4`,
`borescope_frames_<ts>_cam1.csv`). RSI csv and session dir are shared.

ELP bead camera (2026-09-30, `elp=True` / run.py --elp): one ElpCam joins the camera list after
the borescopes; files `elp_<ts>_001_<seg>.mp4` + `elp_frames_<ts>.csv`, `session_meta.json` key
"elp". Borescope file names and meta are unchanged.
"""
import ctypes
import json
import os
import time
from datetime import datetime

import cv2

import config
from borescope import BoreScope
from elp import ElpCam, device_paths, find_device
from rsi_listener import RsiListener


_PREVIEW_WINDOW = "Borescope+RSI Live"
_PREVIEW_SCREEN_W = 1920
_PREVIEW_SCREEN_H = 1020


_ES_CONTINUOUS       = 0x80000000
_ES_SYSTEM_REQUIRED  = 0x00000001
_ES_DISPLAY_REQUIRED = 0x00000002


def prevent_sleep():
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(
            _ES_CONTINUOUS | _ES_SYSTEM_REQUIRED | _ES_DISPLAY_REQUIRED
        )
        print("[PWR] sleep/display prevention ENABLED")


def allow_sleep():
    if os.name == "nt":
        ctypes.windll.kernel32.SetThreadExecutionState(_ES_CONTINUOUS)
        print("[PWR] sleep/display prevention disabled")


def cam_tags(cam_indices):
    """File-name tag per camera: '' for a single-camera session (legacy names), '_cam<idx>'
    when more than one camera records in the same session."""
    if len(cam_indices) == 1:
        return [""]
    return [f"_cam{i}" for i in cam_indices]


def make_session_paths(cam_indices):
    session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_video_dir = os.path.join(config.VIDEO_OUTPUT_DIR, session_ts)
    borescope_csvs = [
        os.path.join(session_video_dir, f"borescope_frames_{session_ts}{tag}.csv")
        for tag in cam_tags(cam_indices)
    ]
    session_rsi_dir = os.path.join(config.CSV_OUTPUT_DIR, session_ts)
    rsi_csv = os.path.join(session_rsi_dir, f"rsi_data_{session_ts}.csv")
    return session_ts, session_video_dir, borescope_csvs, rsi_csv


def latest_rig(cam_indices):
    """Last roll_align.py log entry per camera (Y4F/data/calib/roll_log.jsonl), or {}.
    Gives each session the rig state it was recorded with: roll dev/target, centreline exposure,
    roll-mark note, snapshot path."""
    path = os.path.join(config.Y4F_ROOT, "data", "calib", "roll_log.jsonl")
    rig = {}
    if os.path.exists(path):
        for line in open(path, encoding="utf-8"):
            line = line.strip()
            if line:
                e = json.loads(line)
                if e.get("cam") in cam_indices:
                    rig[e["cam"]] = {k: e.get(k) for k in ("ts", "dev_deg", "target_deg", "centreline_mm",
                                                           "roll_mark", "note", "snapshot")}
    return rig


def write_session_meta(session_dir: str, session_ts: str, start_epoch: float,
                       wait_for_rsi: bool, cam_indices, borescope_csvs,
                       src_name: str = None, elp: dict = None, ceiling_light: str = None):
    tags = cam_tags(cam_indices)
    rig = latest_rig(cam_indices)
    bore = {
        "camera_indices": list(cam_indices),
        "cameras": [
            {"camera_index": idx, "tag": tag, "frames_csv": os.path.basename(csv),
             "rig": rig.get(idx)}
            for idx, tag, csv in zip(cam_indices, tags, borescope_csvs)
        ],
        "backend": "CAP_DSHOW",
        "resolution": [config.RES_W, config.RES_H],
        "fps_nominal": config.FPS_NOMINAL,
        "fourcc": config.VIDEO_FOURCC,
        "segment_minutes": config.SEGMENT_MINUTES,
    }
    if len(cam_indices) == 1:
        bore["camera_index"] = cam_indices[0]   # legacy key, single-camera sessions only
    meta = {
        "session_ts": session_ts,
        "start_epoch": start_epoch,
        "wait_for_rsi": wait_for_rsi,
        "src_name": src_name,
        "ceiling_light": ceiling_light,   # lab ceiling lights on / off (None = not given); user 10-14
        "borescope": bore,
        "rsi": {
            "host": config.RSI_HOST,
            "port": config.RSI_PORT,
            "disconnect_timeout_s": config.RSI_DISCONNECT_TIMEOUT_S,
        },
    }
    if elp is not None:
        meta["elp"] = elp
    os.makedirs(session_dir, exist_ok=True)
    with open(os.path.join(session_dir, "session_meta.json"), "w",
              encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def update_session_meta(session_dir: str, key: str, value):
    """Replace one top-level key of session_meta.json (ELP numbers that exist only at stop)."""
    path = os.path.join(session_dir, "session_meta.json")
    with open(path, encoding="utf-8") as f:
        meta = json.load(f)
    meta[key] = value
    with open(path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def _luma_text(frame):
    """Mean luma of the whole frame and of the lower-left quarter (bed proxy, same ROI as
    analysis/compare_sessions.py luma_bed_ll). Live readout for setting the ring LED."""
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    return f"Luma all {g.mean():5.1f}   bed-LL {g[h // 2:, :w // 2].mean():5.1f}"


def _put(frame, text, y, color, x=20):
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1)


def _draw_preview_hud(frame, borescope, rsi, fps_estimate):
    """HUD on one camera tile. RSI lines are drawn on every tile (same listener)."""
    pkt = rsi.get_latest()
    h = frame.shape[0]
    luma_txt = _luma_text(frame)
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (560, 200), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    _put(frame, f"idx {borescope.cam_index}   Frames {borescope.frame_count}   "
                f"Seg {borescope.segment_idx}", 40, (0, 210, 0))
    _put(frame, f"Capture fps ~ {fps_estimate:5.1f}", 66, (0, 210, 0))

    if pkt is None:
        _put(frame, "RSI: waiting for packets ...", 100, (0, 200, 220))
    else:
        _put(frame, f"RSI pkts {rsi.packet_count}  ipoc {pkt.ipoc}", 100, (0, 210, 0))
        _put(frame, f"Pos X={pkt.rist[0]:8.1f} Y={pkt.rist[1]:8.1f} "
                    f"Z={pkt.rist[2]:7.1f}", 126, (0, 210, 0))
        _put(frame, f"Override {pkt.override:>3} Vel {pkt.vel_act:>4} "
                    f"RPM {pkt.rpm_ext:>4} Delay {pkt.delay}", 152, (0, 210, 0))
    _put(frame, luma_txt, 178, (0, 200, 220))

    cv2.circle(frame, (frame.shape[1] - 40, 30), 12, (0, 0, 220), -1)
    cv2.putText(frame, "REC", (frame.shape[1] - 25, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)

    cv2.putText(frame, "[q] stop session",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
    return frame


def _blank_tile():
    return cv2.UMat(config.RES_H, config.RES_W, cv2.CV_8UC3).get()


def _fit_tile(frame, cam):
    """Borescope frames pass through untouched. An ELP frame (1920x1200) is scaled and
    letterboxed into a RES_W x RES_H tile so the composite and window size stay as they are,
    and turned 180 deg when config.ELP_PROFILE["rot180"] (display only; the mp4 stays raw)."""
    if not isinstance(cam, ElpCam):
        return frame
    h, w = frame.shape[:2]
    s = min(config.RES_W / w, config.RES_H / h)
    frame = cv2.resize(frame, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    dx, dy = config.RES_W - frame.shape[1], config.RES_H - frame.shape[0]
    frame = cv2.copyMakeBorder(frame, dy // 2, dy - dy // 2, dx // 2, dx - dx // 2,
                               cv2.BORDER_CONSTANT)
    return cv2.rotate(frame, cv2.ROTATE_180) if config.ELP_PROFILE["rot180"] else frame


def _composite(tiles):
    """Side-by-side composite of the camera tiles (all RES_W x RES_H)."""
    return tiles[0] if len(tiles) == 1 else cv2.hconcat(tiles)


def _open_preview_window(n_cams):
    cv2.namedWindow(_PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
    cv2.moveWindow(_PREVIEW_WINDOW, 50, 50)
    comp_w = config.RES_W * n_cams
    scale = min(_PREVIEW_SCREEN_W / comp_w, _PREVIEW_SCREEN_H / config.RES_H)
    cv2.resizeWindow(_PREVIEW_WINDOW, int(comp_w * scale), int(config.RES_H * scale))


def run(wait_for_rsi: bool = None, preview: bool = False, src_name: str = None,
        cam_indices=None, elp: bool = None, elp_instance: str = None, ceiling_light: str = None):
    if wait_for_rsi is None:
        wait_for_rsi = config.WAIT_FOR_RSI_DEFAULT
    if cam_indices is None:
        cam_indices = list(config.CAM_INDICES)
    cam_indices = list(cam_indices)
    if len(set(cam_indices)) != len(cam_indices):
        raise ValueError(f"duplicate camera index in {cam_indices}")
    if elp is None:
        elp = config.ELP_DEFAULT
    if not cam_indices and not elp:
        raise ValueError("no camera selected: pass --cam-index IDX [IDX ...] and/or --elp")

    session_ts, video_dir, bore_csvs, rsi_csv = make_session_paths(cam_indices)
    elp_meta = None
    if elp:
        prof = config.ELP_PROFILE
        elp_instance = prof["instance"] if elp_instance is None else elp_instance
        elp_idx, dshow = find_device(prof["name"], elp_instance)
    else:
        dshow = device_paths() if cam_indices else []
    # Unconditional (desktop 10-01): an ELP ("Global Shutter Camera") is never opened as a
    # borescope, with or without --elp; the default CAM_INDICES [1, 2] can hit it.
    bad = [i for i in cam_indices
           if i >= len(dshow) or "global shutter" in dshow[i][0].lower()]
    if bad:
        for i, (n, p) in enumerate(dshow):
            print(f"  [{i}] {n}  usb-instance {p}")
        raise ValueError(f"borescope idx {bad} is an ELP or not a DSHOW device "
                         f"(table above); fix --cam-index")
    if elp:
        elp_csv = os.path.join(video_dir, f"elp_frames_{session_ts}.csv")
        elp_meta = {
            "name": prof["name"], "instance_match": elp_instance,
            "usb_instance": dshow[elp_idx][1], "camera_index": elp_idx,
            "backend": "CAP_DSHOW", "fourcc_in": "MJPG", "resolution": list(prof["res"]),
            "fps_requested": prof["fps"], "fourcc": prof["fourcc"],
            "jpeg_quality": prof["jpeg_quality"] if prof["fourcc"] == "MJPG" else None,
            "segment_minutes": prof["segment_s"][prof["fourcc"]] / 60, "controls": dict(prof["controls"]),
            "stored": "raw sensor orientation", "rot180": prof["rot180"],
            "frames_csv": os.path.basename(elp_csv),
        }
    print(f"[SESSION] {session_ts}")
    print(f"  videos      -> {video_dir}")
    for idx, csv_path in zip(cam_indices, bore_csvs):
        print(f"  borescope   -> idx {idx}: {csv_path}")
    if elp:
        print(f"  elp         -> idx {elp_idx} [{elp_meta['usb_instance']}]: {elp_csv}")
    else:
        print("  elp          = off")
    print(f"  rsi         -> {rsi_csv}")
    print(f"  wait_for_rsi = {wait_for_rsi}")
    print(f"  src_name     = {src_name}")
    print(f"  ceiling_light= {ceiling_light}" + ("   <- NOT GIVEN (--ceiling-light on|off)" if ceiling_light is None else ""))

    prevent_sleep()
    write_session_meta(video_dir, session_ts, time.time(), wait_for_rsi,
                       cam_indices, bore_csvs, src_name, elp_meta, ceiling_light)

    rsi = RsiListener(csv_path=rsi_csv)
    rsi.start()

    # The ELP rides in the same list: ElpCam has BoreScope's interface.
    borescopes = []
    elp_cam = None
    try:
        for idx, tag, csv_path in zip(cam_indices, cam_tags(cam_indices), bore_csvs):
            b = BoreScope(session_ts=session_ts, video_dir=video_dir,
                          csv_path=csv_path, cam_index=idx, tag=tag)
            b.open()
            borescopes.append(b)
        if elp:
            elp_cam = ElpCam(session_ts=session_ts, video_dir=video_dir, csv_path=elp_csv,
                             cam_index=elp_idx, instance=elp_meta["usb_instance"])
            elp_cam.open()
            borescopes.append(elp_cam)
    except Exception:
        for b in borescopes:
            b.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        allow_sleep()
        raise

    if preview:
        _open_preview_window(len(borescopes))

    if wait_for_rsi:
        print("[WAIT] holding borescope recording until RSI connects ...")
        # Live-preview the raw feeds while waiting (nothing is written to disk
        # yet -- start() hasn't spawned the recording threads, so polling
        # each cap directly here is safe, single-threaded access).
        aborted = False
        while not rsi.is_connection_alive():
            if preview:
                tiles = []
                for b in borescopes:
                    ok, frame = b.cap.read()
                    if not ok:
                        frame = _blank_tile()
                    frame = _fit_tile(frame, b)
                    h = frame.shape[0]
                    cv2.putText(frame, f"idx {b.cam_index}  WAITING FOR RSI ...", (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 2)
                    cv2.putText(frame, _luma_text(frame), (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                    cv2.putText(frame, "[q] abort", (20, h - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
                    tiles.append(frame)
                cv2.imshow(_PREVIEW_WINDOW, _composite(tiles))
                key = cv2.waitKey(50) & 0xFF
                if key in (ord('q'), 27):
                    print("\n[STOP] aborted while waiting for RSI")
                    aborted = True
                    break
            else:
                time.sleep(0.2)
        if aborted:
            for b in borescopes:
                b.stop()
            rsi.stop()
            rsi.join(timeout=3.0)
            if preview:
                cv2.destroyAllWindows()
            allow_sleep()
            return
        print("[WAIT] RSI connected -> starting borescope(s).")

    for b in borescopes:
        b.start()

    last_status_t = 0.0
    last_loop_t = time.monotonic()
    last_frame_counts = [0] * len(borescopes)
    fps_estimates = [0.0] * len(borescopes)
    rsi_was_connected = False

    try:
        while True:
            if preview:
                tiles = []
                for b, fps in zip(borescopes, fps_estimates):
                    frame, _ = b.get_latest()
                    if frame is None:
                        frame = _blank_tile()
                    tiles.append(_draw_preview_hud(_fit_tile(frame, b), b, rsi, fps))
                cv2.imshow(_PREVIEW_WINDOW, _composite(tiles))
                key = cv2.waitKey(50) & 0xFF
                if key in (ord('q'), 27):
                    print("\n[STOP] preview quit key")
                    break
            else:
                time.sleep(0.5)

            now = time.monotonic()
            dt = now - last_loop_t
            if dt >= 1.0:
                for k, b in enumerate(borescopes):
                    fps_estimates[k] = (b.frame_count - last_frame_counts[k]) / dt
                    last_frame_counts[k] = b.frame_count
                last_loop_t = now

            if rsi.connected:
                rsi_was_connected = True

            if now - last_status_t >= config.STATUS_PRINT_INTERVAL_S:
                pkt = rsi.get_latest()
                pose_str = "no RSI packet yet"
                if pkt is not None:
                    pose_str = (
                        f"X={pkt.rist[0]:8.1f} Y={pkt.rist[1]:8.1f} "
                        f"Z={pkt.rist[2]:7.1f} ov={pkt.override} "
                        f"vel={pkt.vel_act} rpm={pkt.rpm_ext}"
                    )
                cams_str = " ".join(
                    f"elp{b.cam_index}:{b.frame_count}f/{fps:.0f}fps/seg{b.segment_idx}"
                    f"/drop{b.dropped}" if isinstance(b, ElpCam) else
                    f"idx{b.cam_index}:{b.frame_count}f/{fps:.0f}fps/seg{b.segment_idx}"
                    for b, fps in zip(borescopes, fps_estimates)
                )
                print(f"[STAT] {cams_str} rsi_pkts={rsi.packet_count} | {pose_str}")
                last_status_t = now

            if rsi_was_connected and not rsi.is_connection_alive():
                print("[STOP] RSI disconnected, stopping session.")
                break

    except KeyboardInterrupt:
        print("\n[STOP] keyboard interrupt")

    finally:
        for b in borescopes:
            b.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        if preview:
            cv2.destroyAllWindows()
        allow_sleep()
        if elp_cam is not None:
            update_session_meta(video_dir, "elp", dict(elp_meta, readback=elp_cam.readback,
                                                      measured=elp_cam.summary()))
        print(f"[DONE] session {session_ts}")
        print(f"       videos     : {video_dir}")
        for idx, csv_path in zip(cam_indices, bore_csvs):
            print(f"       borescope  : idx {idx}: {csv_path}")
        if elp_cam is not None:
            print(f"       elp        : idx {elp_cam.cam_index}: {elp_csv}")
        print(f"       rsi        : {rsi_csv}")
