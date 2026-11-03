"""File names and frame-CSV rows of the legacy recorders (no ROS imports).

Legacy references: ``legacy/pipeline_cage1012/elp.py`` (``ElpCam``), ``borescope.py`` (``BoreScope``),
``pipeline.py`` (``cam_tags``, ``make_session_paths``) and ``legacy/realtime_cl_261023/elp_hevc.py``.

- Video segments: ``<prefix>_<ses><tag>_<seg:03d>_<segment start>.<ext>``, e.g.
  ``elp_20261022_165128_001_20261022_165310.avi`` or ``bore_20261022_165128_001_20261022_165310.mp4``.
  ``tag`` is ``""`` for a single camera of its kind (the E1 names) or e.g. ``"_cam1"`` (``pipeline.cam_tags``).
- Frame CSV: ``<csv prefix>_<ses><tag>.csv`` (``elp_frames_<ses>.csv``, ``borescope_frames_<ses>.csv``), written with
  ``csv.writer`` (CRLF line ends, as the E1 files).
- The segment start is local time ``%Y%m%d_%H%M%S`` (legacy: ``datetime.now()`` when the segment was opened).
"""
import csv
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

FRAME_COLS = ("pc_ts_monotonic", "pc_ts_wall", "frame_idx", "segment_idx", "frame_in_segment", "video_file",
              "pc_ts_perf")
CSV_FLUSH_INTERVAL = 200        # config.CSV_FLUSH_INTERVAL

# codec -> file extension (ElpCam: avi for MJPG, mp4 otherwise; elp_hevc: mp4)
EXT = {"hevc": "mp4", "mjpg": "avi", "mp4v": "mp4"}


def local_stamp(t_wall: float) -> str:
    """``YYYYmmdd_HHMMSS`` in local time, as ``datetime.now().strftime`` in the legacy recorders."""
    return datetime.fromtimestamp(t_wall).strftime("%Y%m%d_%H%M%S")


def segment_file_name(prefix: str, session: str, tag: str, segment_idx: int, t_wall_start: float, ext: str) -> str:
    return f"{prefix}_{session}{tag}_{segment_idx:03d}_{local_stamp(t_wall_start)}.{ext}"


def frames_csv_name(csv_prefix: str, session: str, tag: str = "") -> str:
    return f"{csv_prefix}_{session}{tag}.csv"


def recorder_json_name(prefix: str, session: str, tag: str = "") -> str:
    """Sidecar of the recorder (legacy ``elp_recorder_<ses>.json``)."""
    return f"{prefix}_recorder_{session}{tag}.json"


def frame_row(t_mono: float, t_wall: float, frame_idx: int, segment_idx: int, frame_in_segment: int,
              video_file: str, t_perf: float) -> List[object]:
    """One CSV row, formatted as ``ElpCam._write_loop`` / ``BoreScope._run`` format it."""
    return [f"{t_mono:.6f}", f"{t_wall:.6f}", frame_idx, segment_idx, frame_in_segment, video_file, f"{t_perf:.6f}"]


class FramesCsv:
    """The frame CSV: header at open, rows flushed every ``flush_every`` rows and at close."""

    def __init__(self, path: Path, flush_every: int = CSV_FLUSH_INTERVAL):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(FRAME_COLS)
        self.f.flush()
        self.flush_every = max(1, int(flush_every))
        self.rows = 0
        self._n = 0

    def write(self, row: Sequence[object]) -> None:
        self.w.writerow(row)
        self.rows += 1
        self._n += 1
        if self._n >= self.flush_every:
            self.f.flush()
            self._n = 0

    def flush(self) -> None:
        if not self.f.closed:
            self.f.flush()

    def close(self) -> None:
        if not self.f.closed:
            self.f.flush()
            self.f.close()
