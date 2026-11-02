"""
NTG100 borescope continuous capture + chunked mp4 + per-frame CSV.

Pinned to cv2.CAP_DSHOW. Never sets CAP_PROP_EXPOSURE (see memory:
project/borescope_ntg100.md — touching EXPOSURE bricks the driver
until USB replug).

Splits video into SEGMENT_MINUTES chunks to avoid 4-hour single-file
failure observed previously with ZED2i. Per-frame CSV records both
monotonic and wall timestamps for offline merge with the RSI log.
"""
import csv
import os
import threading
import time
from datetime import datetime

import cv2

import config


class BoreScope:
    """One camera. `cam_index` is the DSHOW index; `tag` ("" or e.g. "_cam1") is appended to
    the session ts in every file name so two units recorded in one session do not collide."""

    def __init__(self, session_ts: str, video_dir: str, csv_path: str,
                 cam_index: int, tag: str = ""):
        self.session_ts = session_ts
        self.video_dir = video_dir
        self.csv_path = csv_path
        self.cam_index = cam_index
        self.tag = tag
        self.log = f"[BORE{tag.replace('_', ' ') if tag else ''}]"

        self.cap = None
        self.writer = None
        self.current_video_path = ""
        self.segment_idx = 0
        self.segment_start_mono = 0.0
        self.segment_frame_count = 0
        self.frame_count = 0

        self.csv_file = None
        self.csv_writer = None

        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self._latest_frame = None
        self._latest_idx = 0

    def open(self):
        self.cap = cv2.VideoCapture(self.cam_index, config.CAM_BACKEND)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open borescope at index {self.cam_index} "
                f"(backend={config.CAM_BACKEND})"
            )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.RES_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.RES_H)
        for name, value in config.SAFE_CONTROLS.items():
            prop = getattr(cv2, name, None)
            if prop is not None:
                self.cap.set(prop, value)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_w, actual_h) != (config.RES_W, config.RES_H):
            print(f"{self.log} WARN requested {config.RES_W}x{config.RES_H} "
                  f"but got {actual_w}x{actual_h}")

        os.makedirs(self.video_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "pc_ts_monotonic", "pc_ts_wall", "frame_idx",
            "segment_idx", "frame_in_segment", "video_file", "pc_ts_perf",
        ])
        self.csv_file.flush()
        print(f"{self.log} opened {actual_w}x{actual_h} @ {config.FPS_NOMINAL} fps "
              f"(idx={self.cam_index})")

    def _start_segment(self):
        self.segment_idx += 1
        seg_start_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_video_path = os.path.join(
            self.video_dir,
            f"bore_{self.session_ts}{self.tag}_{self.segment_idx:03d}_{seg_start_ts}.mp4",
        )
        fourcc = cv2.VideoWriter_fourcc(*config.VIDEO_FOURCC)
        self.writer = cv2.VideoWriter(
            self.current_video_path, fourcc, config.FPS_NOMINAL,
            (config.RES_W, config.RES_H),
        )
        self.segment_start_mono = time.monotonic()
        self.segment_frame_count = 0
        print(f"{self.log} segment {self.segment_idx} -> {self.current_video_path}")

    def start(self):
        if self.cap is None:
            raise RuntimeError("open() before start()")
        self._start_segment()
        self.thread = threading.Thread(target=self._run, name=f"BoreScope{self.tag}", daemon=True)
        self.thread.start()

    def _run(self):
        segment_duration_s = config.SEGMENT_MINUTES * 60
        rows_since_flush = 0
        bad_reads = 0

        while not self.stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                bad_reads += 1
                if bad_reads % 30 == 1:
                    print(f"{self.log} WARN frame read failed (total={bad_reads})")
                time.sleep(0.01)
                continue

            t_perf = time.perf_counter()
            t_mono = time.monotonic()
            t_wall = time.time()
            self.frame_count += 1
            self.segment_frame_count += 1
            self.writer.write(frame)

            self.csv_writer.writerow([
                f"{t_mono:.6f}", f"{t_wall:.6f}", self.frame_count,
                self.segment_idx, self.segment_frame_count,
                os.path.basename(self.current_video_path), f"{t_perf:.6f}",
            ])
            rows_since_flush += 1
            if rows_since_flush >= config.CSV_FLUSH_INTERVAL:
                self.csv_file.flush()
                rows_since_flush = 0

            with self.lock:
                self._latest_frame = frame
                self._latest_idx = self.frame_count

            if t_mono - self.segment_start_mono >= segment_duration_s:
                self.writer.release()
                self._start_segment()

    def get_latest(self):
        with self.lock:
            if self._latest_frame is None:
                return None, 0
            return self._latest_frame.copy(), self._latest_idx

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
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
        print(f"{self.log} stopped. frames={self.frame_count} segments={self.segment_idx}")
