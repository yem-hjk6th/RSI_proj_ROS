"""
ELP-USBGS1200P01 (AR0234 global shutter) bead camera: continuous capture + chunked mp4 +
per-frame CSV with the same columns as borescope.py, so the RSI merge works the same way.

What differs from BoreScope (cc_log/reference_elp_gs1200_sensor.md sec. 2-4):
  - selected by DSHOW name + usb-instance substring, never by a bare index: both units report
    the name "Global Shutter Camera" and USB serial 01.00.00, and idx follows neither the
    sensor nor the port;
  - MJPG, size and fps go in the open-time param list: cap.set(FOURCC) after open returns True
    but the graph stays YUY2 (4.9 fps at 1920x1200);
  - manual exposure, fixed gain and white balance (config.ELP_PROFILE["controls"]) are applied
    after every open (the firmware keeps them, but auto-exposure readback is meaningless);
  - a writer thread behind a bounded queue, so the capture loop never waits on the encoder.
    A full queue drops the frame and counts it; frame_idx in the CSV then has a gap.
Frames are written raw (sensor orientation); config.ELP_PROFILE["rot180"] says how to turn them.
"""
import csv
import os
import queue
import threading
import time
from datetime import datetime

import cv2

import config


def device_paths():
    """(FriendlyName, usb-instance) per DSHOW index, e.g. ('Global Shutter Camera',
    '7&325db0ba&0&0000'). Same code as ELP_GS1200/dual_preview.py (desktop 09-29 version, which
    reads DevicePath per device), minus the name-only fallback: without the instance the ELP
    cannot be selected, so errors propagate. Reads the device list only; no stream is opened."""
    from pygrabber.dshow_ids import clsids, DeviceCategories
    from pygrabber.dshow_core import ICreateDevEnum
    from comtypes import client, GUID
    from comtypes.persist import IPropertyBag
    sde = client.CreateObject(clsids.CLSID_SystemDeviceEnum, interface=ICreateDevEnum)
    en = sde.CreateClassEnumerator(GUID(DeviceCategories.VideoInputDevice), dwFlags=0)
    out = []
    m, c = en.Next(1)
    while c > 0:
        bag = m.BindToStorage(0, 0, IPropertyBag._iid_).QueryInterface(IPropertyBag)
        name = bag.Read("FriendlyName", pErrorLog=None)
        try:
            path = bag.Read("DevicePath", pErrorLog=None)
        except Exception:  # noqa: BLE001 - virtual cameras (OBS) have no DevicePath
            path = "-"
        parts = path.split("#")
        out.append((name, parts[2] if len(parts) > 2 else path))
        m, c = en.Next(1)
    return out


def find_device(name: str, instance: str):
    """Print the idx -> name / usb-instance table and return (idx, table) of the one device whose
    name contains `name` and whose usb-instance contains `instance` (both case-insensitive).
    Raises unless exactly one device matches: absent and ambiguous both stop the session."""
    table = device_paths()
    print("[ELP] DSHOW devices:")
    for i, (n, p) in enumerate(table):
        print(f"  [{i}] {n}  usb-instance {p}")
    hits = [i for i, (n, p) in enumerate(table)
            if name.lower() in n.lower() and instance.lower() in p.lower()]
    if len(hits) != 1:
        raise RuntimeError(
            f"[ELP] {len(hits)} DSHOW devices match name '{name}' + usb-instance '{instance}', "
            f"need exactly 1 (table above). Set config.ELP_PROFILE['instance'] or pass "
            f"--elp-instance.")
    return hits[0], table


def open_mjpg(idx, res, fps):
    """ELP_GS1200/dual_preview.py open_mjpg: MJPG / size / fps in the open-time param list."""
    return cv2.VideoCapture(idx, cv2.CAP_DSHOW, [
        cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"),
        cv2.CAP_PROP_FRAME_WIDTH, res[0], cv2.CAP_PROP_FRAME_HEIGHT, res[1],
        cv2.CAP_PROP_FPS, fps, cv2.CAP_PROP_BUFFERSIZE, 1])


class ElpCam:
    """One ELP. Same interface as BoreScope (open / start / get_latest / stop, cam_index,
    frame_count, segment_idx, cap), so pipeline.py drives it in the same camera list.
    Files: elp_<session_ts>_<seg>_<seg_start>.avi (.mp4 with fourcc mp4v) and elp_frames_<session_ts>.csv (both sort
    after bore_* / borescope_*, so the analysis globs that take [0] still get a borescope)."""

    def __init__(self, session_ts: str, video_dir: str, csv_path: str,
                 cam_index: int, instance: str):
        self.p = config.ELP_PROFILE
        self.session_ts = session_ts
        self.video_dir = video_dir
        self.csv_path = csv_path
        self.cam_index = cam_index
        self.instance = instance
        self.log = "[ELP]"

        self.cap = None
        self.writer = None
        self.current_video_path = ""
        self.segment_idx = 0
        self.segment_start_mono = 0.0
        self.segment_frame_count = 0
        self.frame_count = 0          # frames read from the camera
        self.written = 0
        self.dropped = 0              # queue full -> not written
        self.read_fails = 0
        self.readback = {}
        self.fps_open = 0.0
        self.t_first = self.t_last = 0.0

        self.csv_file = None
        self.csv_writer = None

        self.q = queue.Queue(maxsize=self.p["queue_frames"])
        self.thread = None
        self.writer_thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self._latest_frame = None
        self._latest_idx = 0

    def _fail(self, msg):
        self.cap.release()
        self.cap = None
        raise RuntimeError(f"{self.log} {msg}")

    def _wb_prop(self):
        """DSHOW property for the UVC WhiteBalance control (Kelvin). OpenCV builds map it differently:
        laptop cv2 4.13 -> CAP_PROP_WB_TEMPERATURE (WHITE_BALANCE_BLUE_U gives set False / get -1), the
        desktop's build -> the reverse. The mapped one reads back the current value (> 0)."""
        for prop in (cv2.CAP_PROP_WB_TEMPERATURE, cv2.CAP_PROP_WHITE_BALANCE_BLUE_U):
            if self.cap.get(prop) > 0:
                return prop
        return cv2.CAP_PROP_WHITE_BALANCE_BLUE_U

    def open(self):
        w, h = self.p["res"]
        self.cap = open_mjpg(self.cam_index, (w, h), self.p["fps"])
        if not self.cap.isOpened():
            raise RuntimeError(f"{self.log} cannot open DSHOW idx {self.cam_index}")
        props = {name: self._wb_prop() if name == "WB_KELVIN" else getattr(cv2, name)
                 for name, _ in self.p["controls"]}
        if "WB_KELVIN" in props:
            wb = "CAP_PROP_WB_TEMPERATURE" if props["WB_KELVIN"] == cv2.CAP_PROP_WB_TEMPERATURE else "CAP_PROP_WHITE_BALANCE_BLUE_U"
            print(f"{self.log} white balance via {wb}")
        for name, value in self.p["controls"]:
            if not self.cap.set(props[name], value):
                print(f"{self.log} WARN cap.set({name}, {value}) returned False")
        self.readback = {name: self.cap.get(props[name]) for name, _ in self.p["controls"]}
        print(f"{self.log} controls read back: {self.readback}")
        for name, value in self.p["controls"]:
            if name != "CAP_PROP_AUTO_EXPOSURE" and self.readback[name] != value:
                print(f"{self.log} WARN {name} reads back {self.readback[name]}, requested {value}")

        # Warm-up, then a short fps check: YUY2 (MJPG not negotiated) gives ~5 fps at 1920x1200.
        frame = None
        for _ in range(15):
            ok, f = self.cap.read()
            if ok and f is not None:
                frame = f
        t0 = time.perf_counter()
        good = sum(bool(self.cap.read()[0]) for _ in range(30))
        self.fps_open = good / (time.perf_counter() - t0)
        if frame is None or frame.shape[:2] != (h, w):
            self._fail(f"idx {self.cam_index} gave {None if frame is None else frame.shape}, "
                       f"want {w}x{h}")
        if self.fps_open < 0.5 * self.p["fps"]:
            self._fail(f"{self.fps_open:.1f} fps at open (requested {self.p['fps']}): MJPG "
                       f"not negotiated, exposure too long, or USB bandwidth")

        os.makedirs(self.video_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "pc_ts_monotonic", "pc_ts_wall", "frame_idx",
            "segment_idx", "frame_in_segment", "video_file", "pc_ts_perf",
        ])
        self.csv_file.flush()
        print(f"{self.log} opened idx {self.cam_index} [{self.instance}] {w}x{h} MJPG, "
              f"{self.fps_open:.1f} fps measured at open")

    def _start_segment(self):
        self.segment_idx += 1
        seg_start_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        codec = self.p["fourcc"]
        self.current_video_path = os.path.join(
            self.video_dir,
            f"elp_{self.session_ts}_{self.segment_idx:03d}_{seg_start_ts}.{'avi' if codec == 'MJPG' else 'mp4'}",
        )
        fourcc = cv2.VideoWriter_fourcc(*codec)
        if codec == "MJPG":
            # OpenCV's built-in MJPEG writer: one JPEG per frame, no inter-frame coding
            self.writer = cv2.VideoWriter(self.current_video_path, cv2.CAP_OPENCV_MJPEG, fourcc,
                                          self.p["fps"], tuple(self.p["res"]))
            self.writer.set(cv2.VIDEOWRITER_PROP_QUALITY, self.p["jpeg_quality"])
        else:
            self.writer = cv2.VideoWriter(self.current_video_path, fourcc, self.p["fps"],
                                          tuple(self.p["res"]))
        self.segment_start_mono = time.monotonic()
        self.segment_frame_count = 0
        print(f"{self.log} segment {self.segment_idx} -> {self.current_video_path}")

    def start(self):
        if self.cap is None:
            raise RuntimeError("open() before start()")
        self._start_segment()
        if not self.writer.isOpened():
            raise RuntimeError(f"{self.log} VideoWriter did not open {self.current_video_path}")
        self.writer_thread = threading.Thread(target=self._write_loop, name="ElpWriter",
                                              daemon=True)
        self.writer_thread.start()
        self.thread = threading.Thread(target=self._run, name="ElpCam", daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.read_fails += 1
                if self.read_fails % 30 == 1:
                    print(f"{self.log} WARN frame read failed (total={self.read_fails})")
                time.sleep(0.01)
                continue

            t_perf = time.perf_counter()
            t_mono = time.monotonic()
            t_wall = time.time()
            self.frame_count += 1
            if self.frame_count == 1:
                self.t_first = t_wall
            self.t_last = t_wall
            try:
                self.q.put_nowait((frame, t_mono, t_wall, t_perf, self.frame_count))
            except queue.Full:
                self.dropped += 1

            with self.lock:
                self._latest_frame = frame
                self._latest_idx = self.frame_count

    def _write_loop(self):
        segment_duration_s = self.p["segment_s"][self.p["fourcc"]]
        rows_since_flush = 0
        while True:
            item = self.q.get()
            if item is None:
                break
            frame, t_mono, t_wall, t_perf, idx = item
            self.segment_frame_count += 1
            self.writer.write(frame)
            self.written += 1
            self.csv_writer.writerow([
                f"{t_mono:.6f}", f"{t_wall:.6f}", idx,
                self.segment_idx, self.segment_frame_count,
                os.path.basename(self.current_video_path), f"{t_perf:.6f}",
            ])
            rows_since_flush += 1
            if rows_since_flush >= config.CSV_FLUSH_INTERVAL:
                self.csv_file.flush()
                rows_since_flush = 0

            if t_mono - self.segment_start_mono >= segment_duration_s:
                self.writer.release()
                self._start_segment()

    def get_latest(self):
        with self.lock:
            if self._latest_frame is None:
                return None, 0
            return self._latest_frame.copy(), self._latest_idx

    def summary(self):
        """Numbers for session_meta.json at stop; fps from the wall-clock receive times."""
        span = self.t_last - self.t_first
        return {
            "frames_read": self.frame_count, "frames_written": self.written,
            "frames_dropped_queue": self.dropped, "read_fails": self.read_fails,
            "segments": self.segment_idx, "duration_s": round(span, 3),
            "fps_measured": round((self.frame_count - 1) / span, 2) if span > 0 else None,
            "fps_at_open": round(self.fps_open, 2),
        }

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        if self.writer_thread is not None:
            try:
                self.q.put(None, timeout=5.0)   # after the last capture: drain, then exit
            except queue.Full:
                print(f"{self.log} WARN writer not draining; {self.q.qsize()} frames lost")
            self.writer_thread.join(timeout=30.0)
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        print(f"{self.log} stopped. read={self.frame_count} written={self.written} "
              f"dropped={self.dropped} read_fails={self.read_fails} segments={self.segment_idx}")
