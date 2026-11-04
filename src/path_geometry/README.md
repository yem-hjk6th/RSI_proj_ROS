# path_geometry

Task T14 (issue #35). The pattern geometry of the fan and semicircle programs, and the angle failure map tool.
Task T13 (issue #36): the readout gates of these patterns, the path-following ELP grid and the gate log (`gates.py`),
used by `width_readout_elp` and `width_readout_bore1`.
Plain Python: `geometry.py` and `gates.py` use numpy only; the tool adds pandas (and matplotlib for `--plots`). No ROS
imports.

**Nothing here has run on hardware.** The tool works on logged sessions only (`data/`).

| Module | Ported from (`legacy/realtime_cl_cage2_semi_261103/`) | Content |
|---|---|---|
| `geometry.py` | `semi_geom.py` (`Piece`, `PathLine`, `SemiLines`, `SemiPattern`), `fan_geom.py` (`Line`, `FanLines`, `FanPattern`) | `load_pattern(json)`: a `"geometry": "semi"` or `"fan"` pattern and its segs csv (anything else: `ValueError`), with the legacy load-time checks of the segs csv. `Pattern.lines.get(kind, leg)` → a line with `name`, `L`, `project(P) -> (s, e)`, `delta_at`, `curv_at`, `R_at`, `lobe_at`, `t_at`, `n_at`; `Pattern.lines.locate(P, kinds)` |
| `gates.py` | `fan_geom.py` (`_FanReadoutMixin.gate`, `make_fan_bore_gate`, `frame_rows_dir`), `arc_geom.py` (`_ArcReadoutMixin.gate`, `make_arc_bore_gate`), `semi_geom.py` (`PathLine.grid`, `make_semi_bore_gate`, `_GateLog`), `bore1/bore1_rt.py` (`DY_MAX`, `X_LO`, `X_HI`, `LAB_BACK`) | `elp_gate`, `bore_gate`, `elp_grid`, `mdf_band`, the gate log (below) |
| `failure_map.py` | `semi_failure_map_261103.py` | The four tables of one session (below) |
| `cli.py` | – | `angle_failure_map` |
| `plots.py` | – | One PNG per camera |

`patterns/` holds the FAN1 and ARC2 pattern json, segs csv and the ARC2 chord list, byte-identical to the legacy
`pattern/` folder (tested).

## Geometry

Sign conventions as legacy (legacy README, "Geometry and gates"):

- kind `A` = moving −X-ish (ELP-A, bore1), `B` = moving +X-ish (ELP-B, bore2); `leg` counts per kind; `hop` rows are
  connectors, never a leg.
- `delta` = local tangent angle in degrees: B = path angle, A = path angle − 180, wrapped to [−180, 180).
- semi rows: `n` = left normal of the direction of travel, `e` along it; `s` outside [0, L] is clamped by the
  `*_at(s)` functions. `curv` +1 left turn, −1 right turn, 0 straight.
- straight fan lines: `n` from the segs csv (+Y-ish), `curv` 0, `R` inf, `lobe` 0, `delta` = the line's `delta_deg`.
- `locate`: the line of the given kinds with the smallest |e| among those with |e| ≤ tol and −tol ≤ s ≤ L + tol
  (`line_tol_mm`, 1 mm).

## Gates (T13)

`elp_gate(pat, ring, t_frame, kind, lat)` and `bore_gate(pat, ring, t, kind, max_deg, angle_at="tip")` return
`(gate, "")` or `(None, reason)`; `kind` A = ELP-A / bore1, B = ELP-B / bore2. `ring` is anything with `newest_t()` and
`track_at(tq)` (the legacy `RsiRing`, `width_readout_elp.rsi_ring.RsiRing`). Order of checks, reasons and tolerances
as legacy (module docstring):

| | ELP (tip at `t_frame - lat`) | bore (tip at `t`) |
|---|---|---|
| 1 | `stale`: no packet at or after the tip time | same |
| 2 | `no_track`: fewer than two distinct poses | same |
| 3 | `z_or_dir`: `\|z - z_cmd\| > 0.5` or motion along `t(s)` in 0.1 s `< dir_min_mm` | `\|z - z_cmd\| >= 0.5` or `<= 0.3` |
| 4 | `leg_or_xwin`: no line within `line_tol_mm` or x outside `x_window` | `leg_or_x` (A) / `line_or_x` (B): no line, `\|e\| >= 0.5`, or x outside `770 < x < 945` (A) / `x_window` (B) |
| 5 | `min_s_tip`: `s < min_s_tip` | `angle`: `\|delta\| > max_deg + 1e-9` at `s_tip` (`angle_at="tip"`) or `s_tip - 5` (`"obs"`) |

`BoreGate` carries `s_dep = s_tip - 5`, the tested `delta` and `s_angle`, and `obs_before_start` (`s_dep < 0`: the
angle at `s_dep` is the start tangent's, since `delta_at` clamps `s`; 78 passed frames on ARC2-OL, the 78 bore rows
with `s_dep < 0` of the failure map). `elp_grid(line, tip)`: rows at `s_tip - d` (d 12-25 mm, 0.1 mm) along the line
(fan) or the row through the lobe joints (semi; before the leg start the start tangent extended), columns at `e_tip + n`
along the local normal. `GateLog` / `gate_log_row`: the legacy `semi_gate_<ses>.csv` rows (semi patterns only).

## angle_failure_map

```bash
ros2 run path_geometry angle_failure_map --session data/arc2ol_20261103_235936 --out fm_out --plots
# or, without a ROS install:
PYTHONPATH=src/path_geometry python3 -m path_geometry --session data/arc2ol_20261103_235936 --out fm_out
```

Options: `--pattern P` (a json path, or a file name in `patterns/`; default: `realtime.pattern` of
`raw_video/session_meta.json`, by file name), `--out DIR` (default: the session's `roboter/`), `--bin 15` (deg; must
divide 180), `--plots`, `--ses ID` (default: `session_ts` of the session meta). The session layout of `data/`
(`raw_video/`, `roboter/`, `.csv.gz`) and the legacy layout (`raw_video/<ses>/`, `roboter/<ses>/`, `.csv`) are both read.

| Output | Rows |
|---|---|
| `failuremap_bore_<ses>.csv` | Every `ok` frame of `bore1_rt` / `bore2_rt`, geometry at `s_tip` (`tip_*`) and at `s_dep` = s_tip − 5 (`obs_*`). Legacy columns and rows. |
| `failuremap_elp_<ses>.csv` | Every stored row of `readout(B)_rows`, geometry at the row's `s_dep` (`obs_*`). Legacy columns and rows. |
| `failuremap_elpframes_<ses>.csv` | Every gated frame of `readout(B)`, geometry at `s_tip` (`tip_*`). Legacy columns and rows. |
| `failuremap_summary_<ses>.csv` | Per camera × angle bin × curvature sign × R, with a column `basis` (below). |
| `failuremap_<cam>_<ses>.png` (`--plots`) | x = bin centre, y = bore pushed fraction (bores) or ELP row median width with the quartiles (ELPs), one line per (curvature sign, R); `obs` rows only. |

Summary, `basis`:

- `obs`, bores: as legacy. Bin `obs_delta`; group by cam, bin, `obs_curv`, `obs_R`; `n`, `bore_pushed`,
  `bore_sd_med`, `w_med`.
- `obs`, ELPs (**changed from legacy**): every stored ELP row binned by its own `obs_delta`; group by cam, bin,
  `obs_curv`, `obs_R`; `n` = stored rows, `n_frames` = distinct frames, `w_med`, `w_q25`, `w_q75`.
- `tip_frames`, ELPs: the legacy rows (frames binned by `tip_delta`: `n` frames, `elp_frames_with_rows`,
  `elp_rows_med`, `w_med`), kept for comparison only. A frame has no single observed angle.

Why the ELP binning changed (legacy README, known issue 1): the ELP reads 12–25 mm behind the nozzle tip, 35–111 deg
of turn earlier on the ARC2 lobes. The legacy summary binned its ELP frames by the angle at the tip; a lab conclusion
drawn from it was withdrawn. On ARC2-OL, 64 % of the stored ELP rows (98 % of those on a lobe) fall into another
15-deg bin than their frame's tip angle.

Bins: lower edge `floor((delta + 90 + 1e-9) / bin) · bin − 90`, clipped to [−90, 90 − bin] (legacy), so the outer bins
take everything beyond ±90.

## Tests

`pytest test/` (or `colcon test --packages-select path_geometry`).

| Test | Checks |
|---|---|
| `test_geometry_legacy.py` | Acceptance 1: on every ARC2 chord vertex (and segs end point) and on 10 000 random points within 2 mm of the legs of each pattern, `project`, `delta_at`, `curv_at`, `R_at`, `lobe_at`, `t_at`, `n_at` and `locate` equal the legacy `semi_geom` / `fan_geom` classes (floats within 1e-9, ints and names exactly). The pattern copies, the `geometry` refusal, the legacy load-time checks. |
| `test_failure_map_arc2.py` | Acceptance 2–4 on ARC2-OL: bore, ELP-row and ELP-frame tables = the legacy tool's (fixtures); the `obs` bore rows and the `tip_frames` rows of the summary = the legacy summary; the `obs` ELP rows = the groupby written out in the test on the legacy ELP-row table; the legacy summary = the lab's `failuremap_summary_20261103_235936_lab.csv` to its printed precision; the bore `obs_delta` within 0.01 deg of `semi_gate`'s `delta_obs` (3224 frames). The legacy tool is also run live and must still give the fixtures. |
| `test_failure_map_fan1.py` | Acceptance 5 on FAN1-CL: four tables; every row's `tip_delta` / `obs_delta` = its line's `delta_deg`. Session layouts and options. |
| `test_plots.py` | Acceptance 6: the four figures of each session, non-empty. |
| `test_gates_sessions.py` | T13 acceptance 2-4 on both sessions, every logged frame of bore1, bore2, ELP-A, ELP-B: port = legacy offline on the same ring (reason, leg exactly; `s_tip`, `s_dep`, `e` within 1e-9 mm); legacy offline = live (ELP: decision and leg, `s_tip` within 0.001 mm, cuts `t_frame` and `t_done`; bores at the `t_done` cut: reason and leg, `s_tip` within 0.1 mm, every larger difference reproduced by an earlier cut); the gate log = `semi_gate_20261103_235936.csv`; the angle cap for 0-90 deg = legacy with the task file's `angle` counts; `angle_at="obs"`; the 78 frames with `s_dep < 0`. |
| `test_gates_grid.py` | T13 acceptance 5: the grid of 1000 gated tips per pattern = the legacy grid within 1e-9 mm; MDF bands. |
| `test_gates_synthetic.py` | Every reason of both kinds and geometries on synthetic tracks, port = legacy; argument checks; the gate log file. |
| `test_no_ros.py` | No ROS imports; `geometry.py` and `gates.py` need numpy only. |

Fixtures (`test/fixtures/golden_legacy_arc2ol_failuremap_*.csv.gz`, 1.5 MB together) are the legacy tool's four
tables on ARC2-OL. Regenerate them with `python3 src/path_geometry/test/make_golden.py` from the repository root
(cv2 and SciPy are replaced by stand-ins for the legacy imports when missing).
