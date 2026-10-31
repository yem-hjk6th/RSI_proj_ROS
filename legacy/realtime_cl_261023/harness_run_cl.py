"""Desktop integration test of run_cl.py (line C, 2026-10-18): the real run_cl.main with all its threads, a fake ELP
camera (frames of a recorded print, paced to their recorded times, frames skipped when the reader is late like a
BUFFERSIZE-1 camera) and a fake KRC (the recorded RSI packets sent over UDP to 127.0.0.1 at their recorded pace; the
replies are read and parsed). No hardware. The recorder's video writer is replaced by a null writer (no 1 GB/min of
re-encoded video); its output dirs are redirected to harness_out/. The fake KRC does not apply the override, so this
tests wiring, threads, timing and the reply format, not the closed loop itself.
python harness_run_cl.py --pipeline-dir <pipeline_cage1012> --data <Y4F root with data/> --session 20261017_185445
       [--seconds 160] [--mode closed|shadow|step]
"""
import argparse
import re
import socket
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent


class FakeCap:
    """cv2.VideoCapture stand-in. Before go(): returns the first frame at once (open() warm-up and fps check).
    After go(T0): frame i is available at T0 + (t_i - P0); a read returns the newest available frame (older ones
    are grabbed without decoding), waiting for the next one if none is new."""

    def __init__(self, vid_dir, frames, P0, P1):
        self.fr = frames[(frames.pc_ts_perf >= P0) & (frames.pc_ts_perf < P1)].reset_index(drop=True)
        self.vid_dir, self.P0 = Path(vid_dir), P0
        self.props, self.T0, self.i = {}, None, 0
        self.cur_file, self.cap, self.pos = None, None, 0
        first = self._get(0, decode=True)
        self.first = first
        self.served = self.skipped = 0

    def _get(self, i, decode=True):
        r = self.fr.iloc[i]
        if r.video_file != self.cur_file:
            if self.cap is not None:
                self.cap.release()
            self.cap = cv2.VideoCapture(str(self.vid_dir / r.video_file))
            self.cur_file, self.pos = r.video_file, 0
            target = int(r.frame_in_segment) - 1
            if target > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, target)
                self.pos = target
        target = int(r.frame_in_segment) - 1
        while self.pos < target:
            self.cap.grab()
            self.pos += 1
        if decode:
            ok, im = self.cap.read()
        else:
            ok, im = self.cap.grab(), None
        self.pos += 1
        return im if ok else None

    def go(self, T0):
        self.T0 = T0

    def isOpened(self):
        return True

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        return self.props.get(prop, 1.0)

    def read(self):
        if self.T0 is None:
            return True, self.first
        if self.i >= len(self.fr):
            time.sleep(0.05)
            return False, None
        now_rec = self.P0 + (time.perf_counter() - self.T0)
        t = self.fr.pc_ts_perf.to_numpy()
        while self.i + 1 < len(self.fr) and t[self.i + 1] <= now_rec:
            self._get(self.i, decode=False)
            self.i += 1
            self.skipped += 1
        wait = self.T0 + (t[self.i] - self.P0) - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        im = self._get(self.i, decode=True)
        self.i += 1
        self.served += 1
        return (im is not None), im

    def release(self):
        if self.cap is not None:
            self.cap.release()


class NullWriter:
    def __init__(self, *a, **k):
        pass

    def isOpened(self):
        return True

    def set(self, *a):
        return True

    def write(self, frame):
        pass

    def release(self):
        pass


class FakeKRC(threading.Thread):
    """Sends the recorded packets (KRC XML like the real one) to 127.0.0.1:port at their recorded pace, reads replies."""

    def __init__(self, R, P0, port):
        super().__init__(name="FakeKRC", daemon=True)
        self.R, self.P0, self.port = R.reset_index(drop=True), P0, port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.0)
        self.replies, self.sent, self.T0 = [], 0, None

    def go(self, T0):
        self.T0 = T0
        self.start()

    def run(self):
        R = self.R
        t = R.pc_ts_perf.to_numpy()
        for i in range(len(R)):
            wait = self.T0 + (t[i] - self.P0) - time.perf_counter()
            if wait > 0.0015:
                time.sleep(wait - 0.001)
            r = R.iloc[i]
            xml = (f'<Rob Type="KUKA"><RIst X="{r.rist_x_mm:.3f}" Y="{r.rist_y_mm:.3f}" Z="{r.rist_z_mm:.3f}" '
                   f'A="{r.rist_a_deg:.3f}" B="{r.rist_b_deg:.3f}" C="{r.rist_c_deg:.3f}" />'
                   f'<RSol X="{r.rsol_x_mm:.3f}" Y="{r.rsol_y_mm:.3f}" Z="{r.rsol_z_mm:.3f}" '
                   f'A="{r.rsol_a_deg:.3f}" B="{r.rsol_b_deg:.3f}" C="{r.rsol_c_deg:.3f}" />'
                   f'<Delay D="{int(r.delay)}" /><Override>{int(r.override)}</Override><Vel_Act>{int(r.vel_act)}</Vel_Act>'
                   f'<RPM_Ext>{int(r.rpm_ext)}</RPM_Ext><IPOC>{int(r.ipoc)}</IPOC></Rob>')
            self.sock.sendto(xml.encode(), ("127.0.0.1", self.port))
            self.sent += 1
            self._drain()
        t_end = time.perf_counter() + 0.5
        while time.perf_counter() < t_end:
            self._drain()
            time.sleep(0.01)

    def _drain(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, socket.timeout, ConnectionResetError, OSError):
                return
            self.replies.append((time.perf_counter(), data.decode(errors="replace")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pipeline-dir", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--session", required=True)
    ap.add_argument("--seconds", type=float, default=160.0)
    ap.add_argument("--skip", type=float, default=0.0, help="start this many s after the first RSI packet")
    ap.add_argument("--mode", default="closed", choices=["closed", "shadow", "step"])
    ap.add_argument("--press-f", default="", help="comma list of s after start at which the [f] key is pressed")
    ap.add_argument("--extra", default="", help="261020: extra run_cl args, e.g. \"--ov-min 0 --ov-max 100\"")
    ap.add_argument("--out", default=str(HERE / "harness_out"), help="261020: output root (default ./harness_out)")
    ap.add_argument("--port", type=int, default=None, help="261020: UDP port on 127.0.0.1 (default config.RSI_PORT); "
                                                           "another port lets two harnesses run at once")
    a = ap.parse_args()
    sys.path.insert(0, str(HERE))
    sys.path.insert(0, a.pipeline_dir)
    import config
    import elp
    out = Path(a.out)
    config.VIDEO_OUTPUT_DIR = str(out / "raw_video")
    config.CSV_OUTPUT_DIR = str(out / "roboter")
    config.RSI_HOST = "127.0.0.1"
    if a.port is not None:
        config.RSI_PORT = a.port
    data = Path(a.data)
    vid = data / "data" / "raw_video" / a.session
    fr = pd.read_csv(vid / f"elp_frames_{a.session}.csv")
    R = pd.read_csv(data / "data" / "roboter" / a.session / f"rsi_data_{a.session}.csv")
    P0 = float(R.pc_ts_perf.iloc[0]) + a.skip
    P1 = P0 + a.seconds
    fake = FakeCap(vid, fr, P0, P1)
    prof = config.ELP_PROFILE
    elp.device_paths = lambda: [(prof["name"], prof.get("instance") or "fake-instance")]
    elp.open_mjpg = lambda idx, res, fps: fake
    elp.cv2.VideoWriter = NullWriter
    krc = FakeKRC(R[(R.pc_ts_perf >= P0) & (R.pc_ts_perf < P1)], P0, config.RSI_PORT)
    press = [float(x) for x in a.press_f.split(",") if x]

    class FakeMsvcrt:
        """the run_cl main loop reads keys with msvcrt.kbhit / getwch; press [f] at the given times"""
        t0 = None

        @classmethod
        def kbhit(cls):
            if cls.t0 is None or not press:
                return False
            return time.perf_counter() - cls.t0 >= press[0]

        @classmethod
        def getwch(cls):
            press.pop(0)
            return "f"

    sys.modules["msvcrt"] = FakeMsvcrt
    import run_cl
    argv = ["--pipeline-dir", a.pipeline_dir, "--mode", a.mode, "--cam-index", "--src-name", "harness.src",
            "--ceiling-light", "off", "--no-preview"] + a.extra.split()
    th = threading.Thread(target=run_cl.main, args=(argv,), name="run_cl", daemon=True)
    t_start = time.perf_counter()
    th.start()
    time.sleep(4.0)                         # cameras open, UDP bound, waiting for RSI
    T0 = time.perf_counter() + 0.2
    fake.go(T0)
    FakeMsvcrt.t0 = T0
    krc.go(T0)
    krc.join()
    th.join(timeout=20.0)
    ov = [int(m.group(1)) for _, s in krc.replies for m in [re.search(r"<OvCmd>(\d+)</OvCmd>", s)] if m]
    rep = dict(seconds=a.seconds, mode=a.mode, packets_sent=krc.sent, replies=len(krc.replies),
               replies_with_ovcmd=len(ov), ov_values=sorted(set(ov)),
               ov_hist={k: int(v) for k, v in zip(*np.unique(ov, return_counts=True))} if ov else {},
               reply_example=krc.replies[len(krc.replies) // 2][1] if krc.replies else None,
               frames_served=fake.served, frames_skipped_by_fake_camera=fake.skipped,
               run_cl_alive_after=th.is_alive(), wall_s=time.perf_counter() - t_start)
    for k, v in rep.items():
        print(f"{k}: {v}")
    sess = sorted((out / "roboter").glob("*"))
    if sess:
        s = sess[-1]
        oc = list(s.glob("ov_cmd_*.csv"))
        if oc:
            O = pd.read_csv(oc[0])
            print("ov_cmd sidecar rows", len(O), "sources", O.source.value_counts().to_dict())
            print("ov_cmd values", O.ov_cmd.value_counts().sort_index().to_dict())
        for pat in ("ctrl_*.csv", "readout_*.csv"):     # 261020 logs (readout_*.csv also matches readout_rows_*)
            for f in sorted(s.glob(pat)):
                D = pd.read_csv(f)
                print(f"{f.name}: {len(D)} rows, columns {list(D.columns)}")
                if f.name.startswith("ctrl_") and len(D):
                    print("  ctrl src", D.src.value_counts().to_dict(), " ov_pub range", int(D.ov_pub.min()),
                          int(D.ov_pub.max()), " ov_round range", int(D.ov_round.min()), int(D.ov_round.max()),
                          " w_est finite", int(D.w_est.notna().sum()))


if __name__ == "__main__":
    main()
