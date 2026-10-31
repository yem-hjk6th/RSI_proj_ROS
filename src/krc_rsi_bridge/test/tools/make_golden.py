"""Generate the golden fixtures of krc_rsi_bridge from the legacy code (developer tool, not run by CI).

    python3 src/krc_rsi_bridge/test/tools/make_golden.py

Needs numpy, scipy and opencv-python(-headless), because the legacy ``rsi_proc._child`` imports
``rt_controller`` -> ``rt_readout``.

1. ``fixtures/golden_reply_loop.csv.gz``: the legacy reply process itself (``rsi_proc._child``) runs
   on 127.0.0.1 and is fed recorded packets of ``data/e1_20261022_165128`` in lock-step at 4 ms: for
   every packet the generator first writes the shared values (controller OvCmd, its source, heartbeat,
   forced latch) following a fixed script, then sends the packet and waits for the reply. The script
   covers default / control, values outside the window, a stale heartbeat for longer and shorter than
   the ramp, the controller-side "watchdog" source, the forced latch, and the window opening and
   closing. Stored per packet: the inputs, the legacy receive time, OvCmd, source and reply bytes.
2. ``fixtures/golden_path_speed.csv.gz``: legacy ``RsiRing.push`` on recorded packets; v_eff per packet.

Two shims are needed because ``legacy/`` is not complete:

- ``rsi_listener`` (``parse_rsi_xml``, ``build_zero_reply``) is not in ``legacy/``; a minimal
  ElementTree parser stands in for it. It does not take part in the decision.
- In the legacy child ``time.perf_counter`` is rounded to 1 us, so that the receive time written to
  the legacy sidecar csv (6 decimals) is exactly the time the decision used.
"""
import csv
import gzip
import io
import multiprocessing as mp
import socket
import sys
import tempfile
import textwrap
import time
from dataclasses import asdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LEGACY = REPO / "legacy" / "realtime_cl_261023"
DATA = REPO / "data" / "e1_20261022_165128" / "roboter"
FIX = Path(__file__).resolve().parents[1] / "fixtures"
PATTERN = LEGACY / "pattern" / "pattern_usr_ELP_R1v30_CLp_261022.json"
PORT = 59471

STUB = textwrap.dedent('''
    """Stand-in for the legacy rsi_listener module (not in legacy/). Test shim only."""
    import time
    import xml.etree.ElementTree as ET
    from types import SimpleNamespace

    _pc = time.perf_counter
    time.perf_counter = lambda: round(_pc(), 6)     # see make_golden.py docstring

    def build_zero_reply(ipoc):
        return (f'<Sen Type="ImFree"><RKorr X="0.0000" Y="0.0000" Z="0.0000" '
                f'A="0.0000" B="0.0000" C="0.0000" /><IPOC>{ipoc}</IPOC></Sen>').encode()

    def parse_rsi_xml(data, t_mono, t_wall):
        try:
            r = ET.fromstring(data)
            fr = lambda e: [float(e.get(k)) for k in "XYZABC"]
            return SimpleNamespace(ipoc=r.findtext("IPOC").strip(), rist=fr(r.find("RIst")), rsol=fr(r.find("RSol")),
                                   delay=int(r.find("Delay").get("D")), override=int(r.findtext("Override")),
                                   vel_act=int(r.findtext("Vel_Act")), rpm_ext=int(r.findtext("RPM_Ext")),
                                   pc_ts_monotonic=t_mono, pc_ts_wall=t_wall)
        except Exception:
            return None
''')

FRESH, STALE = 1.0e12, -1.0e9      # heartbeat values: always fresh / always stale


def script(j: int):
    """(ov_ctrl, ov_src, hb, forced) for packet j of segment 1 (j = 0 is 250 packets before the window)."""
    k = j - 250
    if k < 500:
        return 40, 0, FRESH, 0                    # controller default
    if k < 1000:
        return -20 + (k % 171), 1, FRESH, 0       # control sweep -20..150 (clamp)
    if k < 1400:
        return 85, 1, FRESH, 0
    if k < 1900:
        return 85, 1, STALE, 0                    # 2 s stale: full ramp 85 -> ov_default
    if k < 2300:
        return 70, 1, FRESH, 0                    # heartbeat back: control again
    if k < 2450:
        return 70, 1, STALE, 0                    # 0.6 s stale: partial ramp
    if k < 2900:
        return 95, 1, FRESH, 0
    if k < 3000:
        return 95, 2, FRESH, 0                    # controller-side "watchdog" source code
    if k < 3500:
        return 60, 1, FRESH, 0
    return 99, 1, FRESH, 1                        # forced latch: ramp 60 -> ov_default


def load_rows():
    with gzip.open(DATA / "rsi_data_20261022_165128.csv.gz", "rt", newline="") as f:
        return list(csv.DictReader(f))


def find_window(rows, cfg):
    from rt_controller import PrintWindow
    from rt_readout import Pattern
    w = PrintWindow(Pattern(str(PATTERN)), cfg)
    i_open = i_close = None
    for i, r in enumerate(rows):
        st = w.update(0.0, (float(r["rist_x_mm"]), float(r["rist_y_mm"]), float(r["rist_z_mm"])))
        if st == "inside" and i_open is None:
            i_open = i
        if st == "after":
            i_close = i
            break
    return i_open, i_close


def packet(r) -> bytes:
    # same layout as legacy harness_run_cl.FakeKRC
    return (f'<Rob Type="KUKA"><RIst X="{float(r["rist_x_mm"]):.3f}" Y="{float(r["rist_y_mm"]):.3f}" '
            f'Z="{float(r["rist_z_mm"]):.3f}" A="{float(r["rist_a_deg"]):.3f}" B="{float(r["rist_b_deg"]):.3f}" '
            f'C="{float(r["rist_c_deg"]):.3f}" />'
            f'<RSol X="{float(r["rsol_x_mm"]):.3f}" Y="{float(r["rsol_y_mm"]):.3f}" Z="{float(r["rsol_z_mm"]):.3f}" '
            f'A="{float(r["rsol_a_deg"]):.3f}" B="{float(r["rsol_b_deg"]):.3f}" C="{float(r["rsol_c_deg"]):.3f}" />'
            f'<Delay D="{int(r["delay"])}" /><Override>{int(r["override"])}</Override>'
            f'<Vel_Act>{int(r["vel_act"])}</Vel_Act><RPM_Ext>{int(r["rpm_ext"])}</RPM_Ext>'
            f'<IPOC>{int(r["ipoc"])}</IPOC></Rob>').encode()


def golden_reply_loop(rows, tmp: Path):
    import rsi_proc
    from rt_controller import CtrlCfg
    # E1 settings: V_max 30 mm/s, v_start 12 -> ov_default 40, speed floor ceil(3.4 x 100 / 30) = 12
    cfg = CtrlCfg(ov_min=12, ov_max=100, ov_default=40, ov_outside=100, fallback_ramp_s=2.0, wd_s=1.0)
    i_open, i_close = find_window(rows, cfg)
    print(f"window opens at row {i_open}, closes at row {i_close}")
    seg1 = list(range(i_open - 250, i_open + 3900))
    seg2 = list(range(i_close - 150, min(len(rows), i_close + 40)))
    sel = seg1 + seg2
    sh = rsi_proc.make_shared(cfg.ov_default)
    params = dict(csv_path=str(tmp / "rsi.csv"), ov_csv_path=str(tmp / "ov.csv"), mode="closed", reply="ov",
                  pattern=str(PATTERN), root=None, host="127.0.0.1", port=PORT, header=["h"], flush_every=1000,
                  sys_path=[str(tmp), str(LEGACY), str(LEGACY / "bore1")], cfg=asdict(cfg), limit_probe=None)
    proc = mp.get_context("fork").Process(target=rsi_proc._child, args=(params, sh), daemon=True)
    proc.start()
    time.sleep(3.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    out = []
    t_next = time.perf_counter()
    for j, i in enumerate(sel):
        ov_ctrl, ov_src, hb, forced = script(j) if j < len(seg1) else (99, 1, FRESH, 1)
        sh["ov_ctrl"].value, sh["ov_src"].value, sh["hb"].value = ov_ctrl, ov_src, hb
        if forced:
            sh["forced"].value = 1
        while time.perf_counter() < t_next:
            pass
        t_next += 0.004
        r = rows[i]
        sock.sendto(packet(r), ("127.0.0.1", PORT))
        reply, _ = sock.recvfrom(4096)
        out.append((r, ov_ctrl, ov_src, hb, forced, reply.decode()))
    sh["stop"].set()
    proc.join(3.0)
    with open(tmp / "ov.csv", newline="") as f:
        side = list(csv.DictReader(f))
    assert len(side) == len(out), (len(side), len(out))
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["ipoc", "rist_x_mm", "rist_y_mm", "rist_z_mm", "rist_a_deg", "rist_b_deg", "rist_c_deg",
                "override", "vel_act", "rpm_ext", "ov_ctrl", "ov_src", "hb", "forced",
                "t_perf", "ov_cmd", "source", "reply"])
    for (r, ov_ctrl, ov_src, hb, forced, reply), s in zip(out, side):
        assert s["ipoc"] == r["ipoc"]
        w.writerow([r["ipoc"], r["rist_x_mm"], r["rist_y_mm"], r["rist_z_mm"], r["rist_a_deg"], r["rist_b_deg"],
                    r["rist_c_deg"], r["override"], r["vel_act"], r["rpm_ext"], ov_ctrl, ov_src, repr(hb), forced,
                    s["pc_ts_perf"], s["ov_cmd"], s["source"], reply])
    (FIX / "golden_reply_loop.csv.gz").write_bytes(gzip.compress(buf.getvalue().encode(), mtime=0))
    srcs = {}
    for s in side:
        srcs[s["source"]] = srcs.get(s["source"], 0) + 1
    print("reply loop fixture:", len(out), "packets, sources", srcs)


def golden_path_speed(rows):
    from rsi_ring import RsiRing
    ring = RsiRing()
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["pc_ts_perf", "ipoc", "rist_x_mm", "rist_y_mm", "rist_z_mm", "v_eff_mm_s"])
    for r in rows[5000:9000]:
        t, ip = float(r["pc_ts_perf"]), float(int(r["ipoc"]))
        xyz = (float(r["rist_x_mm"]), float(r["rist_y_mm"]), float(r["rist_z_mm"]))
        ring.push(t, ip, xyz)
        w.writerow([r["pc_ts_perf"], r["ipoc"], r["rist_x_mm"], r["rist_y_mm"], r["rist_z_mm"],
                    repr(float(ring.ve[ring.n - 1]))])
    (FIX / "golden_path_speed.csv.gz").write_bytes(gzip.compress(buf.getvalue().encode(), mtime=0))
    print("path speed fixture: 4000 packets")


def main():
    sys.path[:0] = [str(LEGACY), str(LEGACY / "bore1")]
    FIX.mkdir(parents=True, exist_ok=True)
    rows = load_rows()
    golden_path_speed(rows)
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        (tmp / "rsi_listener.py").write_text(STUB, encoding="utf-8")
        golden_reply_loop(rows, tmp)


if __name__ == "__main__":
    main()
