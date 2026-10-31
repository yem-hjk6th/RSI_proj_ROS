"""RSI UDP server in its own process (line C, 2026-10-18; replaces the RsiOvListener thread after the field review F2:
in one process the readout / camera threads held the GIL long enough to delay 252-632 of 20 000 replies past the 4 ms
sensor cycle, and a 2 s GIL stall gave 355 late replies in a row > the ETHERNET Timeout of 100).

The child process owns everything the KRC waits for: receive, parse, decide the OvCmd, reply, then log. Its decision
needs nothing from the main process except three shared values the controller writes (OvCmd, its source, a heartbeat):
  - outside the print window (state machine on RIst, rt_controller.PrintWindow, same pattern json) -> ov_outside;
  - forced (operator latch `f`, set by the main process, never cleared) -> ov_default;
  - step mode -> 75/60/90/75 x2, 6 s each from the window opening, then ov_default;
  - closed / shadow -> the controller's OvCmd, or ov_default when the controller heartbeat is older than wd_s;
  - 261022: forced and watchdog do not jump: they ramp linearly from the last value sent inside the window (control /
    default) to ov_default over CtrlCfg.fallback_ramp_s (default 2.0 s; 0 = the old jump; no ramp when nothing was sent
    inside yet); the watchdog ramp ends as soon as the heartbeat is back (control again, as before);
  - every value is clamped to [ov_min, ov_max] (261022: 1..100 outer, ov_min raised to the speed floor; the 261020
    docstring said 50..100) before it is sent (review F1: the KRC LIMIT is the only other
    clamp, and an unaccepted LIMIT parameter would fall back to its factory -10..10); the one exception is the step
    test's --limit-probe: outside the print window it sends that raw value (e.g. 30) so the pendant shows whether the KRC
    LIMIT clamps it to 50 (the only on-site check that LIMIT1 was accepted);
  - any exception in the decision -> ov_default ("fallback").
Reply "ov" (closed / step) = the original zero-RKorr reply with <OvCmd>NN</OvCmd>; "zero" (shadow) = byte-identical to
pipeline_cage1012.rsi_listener.build_zero_reply. Same rsi_data_<ses>.csv as the original listener + sidecar
ov_cmd_<ses>.csv (pc_ts_perf, ipoc, ov_cmd, source). Every packet (t_perf, ipoc, x, y, z) also goes to a queue that a
thread in the main process drains into the RsiRing (readout / controller history); a full queue drops, it never blocks.
Ctrl+C in the console ends this process at once (KeyboardInterrupt) -> no more replies -> the KRC stops on its Timeout.
The child raises its own priority to ABOVE_NORMAL (Windows) when it can.
"""
import csv
import logging
import multiprocessing as mp
import os
import queue
import socket
import sys
import threading
import time

log = logging.getLogger("rsi_proc")

SRC = {0: "default", 1: "control", 2: "watchdog"}          # controller-side sources (shared int)
SIDECAR_HEADER = ["pc_ts_perf", "ipoc", "ov_cmd", "source"]


def build_ov_reply(ipoc: str, ov: int) -> bytes:
    return (
        f'<Sen Type="ImFree">'
        f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
        f'A="0.0000" B="0.0000" C="0.0000" />'
        f"<OvCmd>{int(ov)}</OvCmd>"
        f"<IPOC>{ipoc}</IPOC>"
        f"</Sen>"
    ).encode()


def make_shared(ov_default: int):
    # lock=False (review r2): every value has one writer and is a single aligned 1/4/8-byte load / store; with the
    # default lock, a main-process thread that holds it while waiting for its own GIL would stall the reply
    def v(code, init):
        return mp.Value(code, init, lock=False)
    return dict(
        ov_ctrl=v("i", int(ov_default)), ov_src=v("i", 0), hb=v("d", 0.0),
        forced=v("b", 0), stop=mp.Event(),
        packets=v("q", 0), parse_fail=v("q", 0), q_drop=v("q", 0), errors=v("q", 0),
        connected=v("b", 0), last_mono=v("d", 0.0), win_state=v("i", 0),
        last_ov=v("i", -1), last_src=v("i", -1),
        q=mp.Queue(maxsize=50000),
    )


SRC_OUT = ["outside", "forced", "step", "default", "control", "watchdog", "fallback", "none", "limit_probe"]


def _child(params, sh):
    """runs in the RSI process"""
    for p in params["sys_path"]:
        if p not in sys.path:
            sys.path.insert(0, p)
    # WARNING (review r2): no console write between a packet and its reply (a console in QuickEdit selection blocks
    # every write -> no reply -> KRC Timeout); window OPEN / CLOSED is printed by the main process
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s rsi_proc: %(message)s")
    parent = mp.parent_process()
    try:
        import ctypes
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x8000)  # ABOVE_NORMAL
    except Exception:  # noqa: BLE001
        pass
    from rsi_listener import build_zero_reply, parse_rsi_xml
    from rt_controller import CtrlCfg, PrintWindow
    from rt_readout import Pattern

    cfg = CtrlCfg(**params["cfg"])
    win = PrintWindow(Pattern(params["pattern"], params["root"]), cfg)
    lo, hi = int(cfg.ov_min), int(cfg.ov_max)

    def clamp(v):
        return min(max(int(v), lo), hi)

    mode, reply = params["mode"], params["reply"]
    probe = params.get("limit_probe") if mode == "step" else None
    step_t0 = None
    ramp_s = float(cfg.fallback_ramp_s) if cfg.fallback_ramp_s and cfg.fallback_ramp_s > 0 else 0.0   # 261022
    fb = None             # 261022: (t0, ov_from) of the running forced / watchdog ramp
    ov_last_in = None     # 261022: last value sent inside the window under control / default (the ramp start)
    os.makedirs(os.path.dirname(params["csv_path"]), exist_ok=True)
    f_rsi = open(params["csv_path"], "w", newline="", encoding="utf-8")
    w_rsi = csv.writer(f_rsi)
    w_rsi.writerow(params["header"])
    f_ov = open(params["ov_csv_path"], "w", newline="", encoding="utf-8")
    w_ov = csv.writer(f_ov)
    w_ov.writerow(SIDECAR_HEADER)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.5)
    sock.bind((params["host"], params["port"]))
    log.warning("listening on %s:%s (mode=%s, reply=%s, ov window %d..%d)", params["host"], params["port"], mode,
                reply, lo, hi)
    rows = 0
    q = sh["q"]
    try:
        while not sh["stop"].is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                if parent is not None and not parent.is_alive():
                    break
                continue
            except OSError:
                break
            t_perf = time.perf_counter()
            t_mono = time.monotonic()
            t_wall = time.time()
            sh["packets"].value += 1
            sh["last_mono"].value = t_mono
            if not sh["connected"].value:
                log.info("connected to %s", addr)
                sh["connected"].value = 1
            pkt = parse_rsi_xml(data, t_mono, t_wall)
            if pkt is None:
                sh["parse_fail"].value += 1
                continue
            # ---- decide (must not raise) and reply first
            try:
                st = win.update(t_perf, pkt.rist[:3])
                if st != "inside":
                    ov, src = cfg.ov_outside, 0
                elif sh["forced"].value:
                    ov, src = cfg.ov_default, 1
                elif mode == "step":
                    if step_t0 is None:
                        step_t0 = t_perf
                    k = int((t_perf - step_t0) / cfg.step_hold_s)
                    ov, src = (cfg.step_seq[k] if k < len(cfg.step_seq) else cfg.ov_default), 2
                elif t_perf - sh["hb"].value > cfg.wd_s:
                    ov, src = cfg.ov_default, 5
                else:
                    ov, src = sh["ov_ctrl"].value, 3 + sh["ov_src"].value   # default / control / watchdog
                # 261022: forced (1) / watchdog (5): ramp from the last inside value to ov_default
                if src in (1, 5) and ramp_s > 0.0 and ov_last_in is not None:
                    if fb is None:
                        fb = (t_perf, ov_last_in)
                    f = min(1.0, (t_perf - fb[0]) / ramp_s)
                    ov = int(round(fb[1] + f * (cfg.ov_default - fb[1])))
                elif src not in (1, 5):
                    fb = None
                ov = clamp(ov)
                if src in (3, 4):
                    ov_last_in = ov
                if probe is not None and st != "inside":
                    ov, src = probe, 8     # step mode only: deliberately outside 50..100, the KRC LIMIT must clamp it
            except Exception:  # noqa: BLE001
                sh["errors"].value += 1
                ov, src = clamp(cfg.ov_default), 6
            if reply == "ov":
                sock.sendto(build_ov_reply(pkt.ipoc, ov), addr)
            else:
                sock.sendto(build_zero_reply(pkt.ipoc), addr)
            # ---- then share and log
            sh["win_state"].value = {"before": 0, "inside": 1, "after": 2}.get(win.state, 0)
            sh["last_ov"].value, sh["last_src"].value = ov, src
            try:
                q.put_nowait((t_perf, pkt.ipoc, pkt.rist[0], pkt.rist[1], pkt.rist[2]))
            except queue.Full:
                sh["q_drop"].value += 1
            w_rsi.writerow([
                f"{pkt.pc_ts_monotonic:.6f}", f"{pkt.pc_ts_wall:.6f}", pkt.ipoc,
                f"{pkt.rist[0]:.3f}", f"{pkt.rist[1]:.3f}", f"{pkt.rist[2]:.3f}",
                f"{pkt.rist[3]:.3f}", f"{pkt.rist[4]:.3f}", f"{pkt.rist[5]:.3f}",
                f"{pkt.rsol[0]:.3f}", f"{pkt.rsol[1]:.3f}", f"{pkt.rsol[2]:.3f}",
                f"{pkt.rsol[3]:.3f}", f"{pkt.rsol[4]:.3f}", f"{pkt.rsol[5]:.3f}",
                pkt.delay, pkt.override, pkt.vel_act, pkt.rpm_ext, f"{t_perf:.6f}",
            ])
            w_ov.writerow([f"{t_perf:.6f}", pkt.ipoc, ov, SRC_OUT[src]])
            rows += 1
            if rows >= params["flush_every"]:
                f_rsi.flush()
                f_ov.flush()
                rows = 0
                if parent is not None and not parent.is_alive():   # review r2: main process crashed -> stop replying
                    log.warning("main process gone: RSI replies stop now (the KRC stops the robot on its Timeout)")
                    break
    except KeyboardInterrupt:
        log.warning("Ctrl+C: RSI replies stop now (the KRC stops the robot on its Timeout)")
    finally:
        try:
            sock.close()
        except OSError:
            pass
        f_rsi.flush()
        f_rsi.close()
        f_ov.flush()
        f_ov.close()
        q.cancel_join_thread()       # review r2: with the main process gone nobody drains q; do not hang on exit
        log.info("stopped. packets=%d parse_fail=%d errors=%d queue_drops=%d", sh["packets"].value,
                 sh["parse_fail"].value, sh["errors"].value, sh["q_drop"].value)


class RsiProc:
    """Main-process handle: starts the child, drains its packet queue into the RsiRing (and the controller's own copy of
    the window state machine), and gives run_cl.py the same status fields as the old listener."""

    def __init__(self, csv_path, ov_csv_path, ring, ctrl, mode, pattern, root, host, port, header, flush_every,
                 sys_path, limit_probe=None):
        from dataclasses import asdict
        self.ring, self.ctrl = ring, ctrl
        self.sh = make_shared(ctrl.cfg.ov_default)
        self.params = dict(csv_path=csv_path, ov_csv_path=ov_csv_path, mode=mode,
                           reply="ov" if mode in ("closed", "step") else "zero", pattern=str(pattern),
                           root=None if root is None else str(root), host=host, port=int(port), header=list(header),
                           flush_every=int(flush_every), sys_path=list(sys_path), cfg=asdict(ctrl.cfg),
                           limit_probe=None if limit_probe is None else int(limit_probe))
        self.proc = mp.Process(target=_child, args=(self.params, self.sh), name="RsiProc", daemon=True)
        self._drain_stop = threading.Event()
        self.n_drain_err = 0
        self._drain = threading.Thread(target=self._drain_loop, name="RsiDrain", daemon=True)
        ctrl.publish = self._publish

    def _publish(self, ov, src_code, hb):
        self.sh["ov_ctrl"].value = int(ov)
        self.sh["ov_src"].value = int(src_code)
        self.sh["hb"].value = float(hb)

    def force_open_loop(self):
        self.sh["forced"].value = 1

    def _drain_loop(self):
        q = self.sh["q"]
        while not self._drain_stop.is_set():
            try:
                t, ipoc, x, y, z = q.get(timeout=0.2)
            except queue.Empty:
                continue
            except (EOFError, OSError):
                break
            try:
                ipoc_n = float(int(ipoc))
            except ValueError:
                ipoc_n = 0.0
            # review fix r1 (S6): a packet that makes push / win.update raise must not kill this thread (a dead drain froze
            # the ring while the controller heartbeat stayed fresh); the controller also stops its heartbeat when the
            # ring's newest packet is older than CtrlCfg.ring_fresh_s
            try:
                self.ring.push(t, ipoc_n, (x, y, z))
                self.ctrl.win.update(t, (x, y, z))
            except Exception:  # noqa: BLE001
                self.n_drain_err += 1
                if self.n_drain_err <= 5:
                    log.exception("RsiDrain: packet ipoc %s not pushed (%d so far)", ipoc, self.n_drain_err)

    def start(self):
        if sys.platform == "win32":
            import multiprocessing.popen_spawn_win32 as pw
            if not hasattr(pw.msvcrt, "open_osfhandle"):     # a test harness replaced sys.modules["msvcrt"] (key fakes)
                import importlib.machinery
                import importlib.util
                spec = importlib.machinery.BuiltinImporter.find_spec("msvcrt")
                real = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(real)
                pw.msvcrt = real
        self.proc.start()
        self._drain.start()

    def stop(self):
        self.sh["stop"].set()

    def join(self, timeout=None):
        self.proc.join(timeout)
        self._drain_stop.set()
        self._drain.join(timeout=2.0)

    # status (same names as RsiListener)
    @property
    def packet_count(self):
        return self.sh["packets"].value

    @property
    def connected(self):
        return bool(self.sh["connected"].value)

    @property
    def parse_fail_count(self):
        return self.sh["parse_fail"].value

    def is_connection_alive(self, timeout_s=2.0):
        lm = self.sh["last_mono"].value
        return lm > 0 and (time.monotonic() - lm) < timeout_s

    @property
    def last_reply(self):
        s = self.sh["last_src"].value
        return self.sh["last_ov"].value, (SRC_OUT[s] if 0 <= s < len(SRC_OUT) else "none")
