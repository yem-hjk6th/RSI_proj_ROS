# path_geometry

Task T14 (issue #35). The pattern geometry of the fan and semicircle programs, and the angle failure map tool.
Plain Python: `geometry.py` uses numpy only; the tool adds pandas (and matplotlib for `--plots`). No ROS imports.

**Nothing here has run on hardware.** The tool works on logged sessions only (`data/`).

| Module | Ported from (`legacy/realtime_cl_cage2_semi_261103/`) | Content |
|---|---|---|
| `geometry.py` | `semi_geom.py` (`Piece`, `PathLine`, `SemiLines`, `SemiPattern`), `fan_geom.py` (`Line`, `FanLines`, `FanPattern`) | `load_pattern(json)`: a `"geometry": "semi"` or `"fan"` pattern and its segs csv (anything else: `ValueError`), with the legacy load-time checks of the segs csv. `Pattern.lines.get(kind, leg)` → a line with `name`, `L`, `project(P) -> (s, e)`, `delta_at`, `curv_at`, `R_at`, `lobe_at`, `t_at`, `n_at`; `Pattern.lines.locate(P, kinds)` |
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
| `test_no_ros.py` | No ROS imports; `geometry.py` needs numpy only. |

Fixtures (`test/fixtures/golden_legacy_arc2ol_failuremap_*.csv.gz`, 1.5 MB together) are the legacy tool's four
tables on ARC2-OL. Regenerate them with `python3 src/path_geometry/test/make_golden.py` from the repository root
(cv2 and SciPy are replaced by stand-ins for the legacy imports when missing).
