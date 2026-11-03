"""Regenerate the golden fixture: the LEGACY bore1 thread on session E1 with the replay head.

Run from the repository root (needs numpy and pandas; cv2 and SciPy are replaced by stand-ins when missing):
    python3 src/width_readout_bore1/test/make_golden.py

The legacy ``bore1_rt.Bore1Thread`` runs unchanged (its gate, its row logic, its csv writer) with:
- a camera stand-in that hands it the frames of E1's bore1_rt log one by one, at their frame times from
  ``borescope_frames_<ses>.csv.gz``; before it hands over frame k it pushes the RSI packets received at or before
  ``t_gate`` of frame k into the legacy ``RsiRing`` (the schedule of ``width_readout_bore1.replay``);
- the replay head (``width_readout_bore1.head.ReplayHead`` on the same log), called through the legacy ``head(frame)``;
- the legacy ``rt_readout.Pattern`` of the legacy pattern folder.

Writes ``test/fixtures/golden_legacy_e1_bore1.csv.gz``: the csv the legacy thread wrote (``B1_COLS``, legacy formats;
``t_done`` blanked, it is the wall time of this run), plus the unrounded gate outputs ``s_tip_u, s_dep_u, x_u, y_u,
z_u`` (repr of the floats: s_tip and the tip from the legacy ``bore1_gate``, s_dep = s_tip - LAB_BACK as the legacy
row computes it; the pushed rows' s_dep, d and w are checked against what the thread pushed to its row store).
The gzip header carries no time, so the file is the same on every run.
"""
import csv
import gzip
import io
import sys
import tempfile
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from legacy_paths import BORE1_RT, FIXTURE, FRAMES, LEGACY_DIR, LEGACY_PATTERN, RSI, import_legacy  # noqa: E402
from width_readout_bore1.head import ReplayHead  # noqa: E402
from width_readout_bore1.replay import load_rsi, schedule  # noqa: E402

U_COLS = ["s_tip_u", "s_dep_u", "x_u", "y_u", "z_u"]


class _Frame:
    """What the legacy thread gets as the frame: only the replay head looks at it (for the frame time)."""

    def __init__(self, t):
        self.t = t


class _Cam:
    """Legacy BoreScopeRT.get_latest_rt stand-in: the next scheduled frame on every call, ring filled first."""

    def __init__(self, steps, ring, pk):
        self.steps, self.ring, self.pk = steps, ring, pk
        self.k, self.ptr = 0, 0
        self.done = threading.Event()

    def get_latest_rt(self):
        if self.k >= len(self.steps):
            self.done.set()                     # the thread finished the last frame and asks for the next one
            return None, 0, 0.0
        st = self.steps[self.k]
        t, ip, xyz = self.pk
        while self.ptr < len(t) and t[self.ptr] <= st.t_gate:
            self.ring.push(t[self.ptr], ip[self.ptr], xyz[self.ptr])
            self.ptr += 1
        self.k += 1
        return _Frame(st.t_frame), st.frame, st.t_frame


def run_legacy(wait_s: float = 0.06):
    """Run the legacy thread on E1. Returns (rows of its csv as lists of strings, {frame: gate result},
    {frame: pushed readout}, the thread)."""
    rt_readout, rsi_ring, bore1_rt = import_legacy()
    pat = rt_readout.Pattern(LEGACY_PATTERN, LEGACY_DIR)
    ring = rsi_ring.RsiRing()
    steps = schedule(BORE1_RT, FRAMES)
    cam = _Cam(steps, ring, load_rsi(RSI))
    replay = ReplayHead(BORE1_RT)
    gates, pushed = {}, {}
    cur = {}
    orig_gate = bore1_rt.bore1_gate

    def gate(p, r, t):                          # the legacy gate, recorded
        g, why = orig_gate(p, r, t)
        gates[cur["frame"]] = (g, why)
        return g, why

    def head(frame):                            # the legacy call head(frame) -> the replay head at the frame time
        return replay(None, frame.t)

    store = bore1_rt.RowStore("bore1", bore1_rt.LAB_BACK)
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "bore1_rt_legacy.csv"
        def sink(ro, now):                      # what the thread pushed to the controller's row store
            pushed[ro.frame] = ro
        th = bore1_rt.Bore1Thread(cam, head, pat, ring, store, out, sink=sink, wait_s=wait_s)
        orig_get = cam.get_latest_rt

        def get():                              # remember which frame the gate is called for
            fr, idx, t = orig_get()
            if fr is not None:
                cur["frame"] = idx
            return fr, idx, t
        cam.get_latest_rt = get
        bore1_rt.bore1_gate = gate
        try:
            th.start()
            if not cam.done.wait(600):
                raise TimeoutError("legacy Bore1Thread did not finish")
            th.stop_event.set()
            th.join(10)
        finally:
            bore1_rt.bore1_gate = orig_gate
        with open(out, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
    assert rows[0] == bore1_rt.B1_COLS and th.errors == 0, (rows[0], th.errors)
    return rows, gates, pushed, th


def fixture_rows(rows, gates, pushed):
    """Header + rows of the fixture (see the module docstring)."""
    from width_readout_bore1.gate import LAB_BACK
    cols = rows[0]
    i_done, i_gate = cols.index("t_done"), cols.index("gate")
    out = [cols + U_COLS]
    for r in rows[1:]:
        fr = int(r[0])
        g, why = gates[fr]
        r = list(r)
        r[i_done] = ""
        if r[i_gate] != "ok":
            assert g is None and why == r[i_gate]
            out.append(r + [""] * len(U_COLS))
            continue
        s_dep = g.s_tip - LAB_BACK
        ro = pushed.get(fr)
        if ro is not None:
            assert ro.rows[0, 0] == LAB_BACK and ro.rows[0, 1] == s_dep and ro.s_tip == g.s_tip
        assert (ro is not None) == (r[-1] == "1"), fr
        out.append(r + [repr(float(v)) for v in (g.s_tip, s_dep, g.tip[0], g.tip[1], g.tip[2])])
    return out


def write_fixture(lines, path=FIXTURE):
    buf = io.StringIO()
    csv.writer(buf, lineterminator="\n").writerows(lines)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz:
        gz.write(buf.getvalue().encode("utf-8"))


def main():
    rows, gates, pushed, th = run_legacy()
    lines = fixture_rows(rows, gates, pushed)
    write_fixture(lines)
    ok = sum(1 for r in lines[1:] if r[3] == "ok")
    print(f"{len(lines) - 1} rows ({ok} ok, {len(pushed)} pushed, thread frames {th.frames}, predicted "
          f"{th.predicted}) -> {FIXTURE} ({FIXTURE.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
