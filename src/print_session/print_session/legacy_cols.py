"""Column names, order and number formats of the legacy session logs. No ROS imports.

- rsi_data_<ses>.csv: header of the E1 file (written by the pipeline's RsiListener.HEADER, not in legacy/), values as
  rsi_proc.py writes them (times %.6f, RIst / RSol %.3f, integers as they are).
- ov_cmd_<ses>.csv: rsi_proc.SIDECAR_HEADER.
- ctrl_ / readout_ / readout_rows_<ses>.csv: CTRL_COLS / READOUT_COLS / ROWS_COLS of run_cl.py, values through run_cl._f.
"""
import math

from cl_controller.record import CTRL_COLS  # noqa: F401  (re-exported: the one list of the ctrl columns)

RSI_COLS = ["pc_ts_monotonic", "pc_ts_wall", "ipoc", "rist_x_mm", "rist_y_mm", "rist_z_mm", "rist_a_deg", "rist_b_deg",
            "rist_c_deg", "rsol_x_mm", "rsol_y_mm", "rsol_z_mm", "rsol_a_deg", "rsol_b_deg", "rsol_c_deg", "delay",
            "override", "vel_act", "rpm_ext", "pc_ts_perf"]
OV_COLS = ["pc_ts_perf", "ipoc", "ov_cmd", "source"]
READOUT_COLS = ["frame", "t_frame_perf", "t_done_perf", "compute_ms", "skipped_total", "leg", "s_tip", "n_rows",
                "n_used", "w_med_used"]
ROWS_COLS = ["frame", "d", "s_dep", "w"]

# ctrl columns -> decimals of run_cl.log_update (_f(x, nd)); t_perf / t_wall are written with "%.6f"
CTRL_ND = dict(s_tip=3, veff=3, w_est=4, rhat=5, rhat_f=5, logk=5, slow=5, k=5, ov_raw=3, rows_age_s=3, rhat_elp=5,
               lag_elp=3, rhat_b1=5, lag_b1=3, e=5, du=5, u=5, lnk_ff=5, v_tip=3, w_target=3, lnk_tgt=5, nobed_s=2,
               lnk_ol=5, du_blind=5, t_blind=2)
CTRL_BOOL = ("on_a", "have_est", "forced")
CTRL_TRI = ("valid_elp", "valid_b1", "hold_bed")

# Columns no recorded topic carries. The exporter fills them as stated; the round-trip test checks this list.
NOT_CARRIED = {
    "rsi_data": {
        "pc_ts_monotonic": "not carried: written as pc_ts_perf (on Linux both are CLOCK_MONOTONIC; the E1 values "
                           "came from a 15.6 ms Windows tick and differ)",
        **{f"rsol_{a}": "not carried (RSol is parsed but not in the packet ring): empty"
           for a in ("x_mm", "y_mm", "z_mm", "a_deg", "b_deg", "c_deg")},
    },
    "readout": {
        "frame": "not carried (Width has no frame index): frames numbered 1, 2, ... in stamp order; the same numbers "
                 "are used in readout_rows",
        "t_frame_perf": "derived: Width.stamp (capture time, wall clock) minus the median KrcState.stamp - "
                        "ReplyRecord.pc_ts_perf offset of the bag",
        "t_done_perf": "not carried: empty",
        "compute_ms": "not carried: empty",
        "skipped_total": "not carried: empty",
        "n_rows": "not carried (only used rows are published): empty",
        "_rows": "frames the readout gated out (no Width published) are not in the export",
    },
    "readout_rows": {
        "frame": "renumbered as in readout",
    },
}


def f6(x) -> str:
    return f"{float(x):.6f}"


def f3(x) -> str:
    return f"{float(x):.3f}"


def fnd(x, nd):
    """run_cl._f: '' for None / NaN / inf, else round(x, nd) (written by csv as repr of the float)."""
    return "" if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(float(x), nd)
