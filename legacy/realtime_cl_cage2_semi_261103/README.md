# realtime_cl_cage2_semi_261103 (reference only)

The lab pipeline as it ran on 2026-11-03 for the diagonal fan **FAN1** and the semicircle meander **ARC2** on cage 2
(two ELP cameras, ELP-A and ELP-B, and two borescopes, bore1 and bore2). Only the modules that tasks T13 and T14 need
are here.

**Lineage (lab copies, each a sibling folder):** two-ELP base (2026-11-02) → bore-dual copy (adds bore2) → `fan`
(`fan_geom.py`) → `fanbore` (bore gates on the fan lines) → `arc` / `arcbore` (`arc_geom.py`) → `semi` (`semi_geom.py`,
the gate log and the failure map). `fan_geom.py`, `arc_geom.py`, `rt_readout.py` and the bore modules are byte-identical
through `fanbore`, `arcbore` and `semi`. The lab checked that a deterministic replay of FAN1-CL gives byte-identical
outputs through all three copies, so this one folder serves both sessions in `data/`.

Same publication rewrites as `../README.md` (identifiers, comments, the `usr_` prefix). The scripts do not run
unmodified from this folder; the logic is unchanged.

| File | Role |
|---|---|
| `fan_geom.py` | Straight print lines at any angle (FAN1): line lookup by projection, the tangent-following ELP grid, `FanRowStore`, `FanController`, `make_fan_bore_gate(kind, max_deg)` |
| `arc_geom.py` | Circular arcs (ARC1, not in this repository): the same interface on arcs; `make_arc_bore_gate` is the bore gate that `semi_geom.py` reuses |
| `semi_geom.py` | ARC2: one leg = one row = a chain of lines and arcs (`Piece`, `PathLine`, `SemiLines`, `SemiPattern`), the path-following ELP grid, `make_semi_bore_gate`, the gate log |
| `rt_readout.py` | The cage-2 ELP readout these modules extend (`Pattern`, `Gate`, `RtReadout`, `RtReadoutB`, grid constants) |
| `bore1/bore1_rt.py`, `bore1/bore2_rt.py` | The cage-2 borescope threads; `bore1_gate` / `bore2_gate` are the module-level gates that `run_cl.py` replaces for these patterns |
| `semi_failure_map_261103.py` | The angle failure map of one session (four tables; see "Known issues"). It runs as is: `--data <dir> --ses 20261103_235936` on `data/arc2ol_20261103_235936/` decompressed into `<dir>/raw_video/<ses>/` and `<dir>/roboter/<ses>/` reproduces the lab's four tables byte for byte |
| `replay_cage2.py` | The lab's causal offline replay (RSI packets, controller ticks, ELP frames in time order) |
| `test_semi_synth_261103.py`, `test_fan_synth_261103.py` | The lab's synthetic checks (they import `run_cl.py`, which is not here) |
| `pattern/` | FAN1 and ARC2 pattern json + segs csv (ARC2 also its chord list) |

The two print programs are in `../krc_config/krl/` (`usr_ELP_FAN1_OL_261103.src`, `usr_ELP_ARC2_OL_261103.src`). Their
headers still say `DRAFT, NOT RUN`; both ran in the lab on 2026-11-03 (FAN1 open and closed loop, ARC2 open and closed
loop). The published copies differ from the lab files only in the rewritten names and comments, so their checksums
differ from the ones quoted in the lab notes.

## How `run_cl.py` of this copy uses the modules (the file is not included)

- A pattern json with `"geometry"` in `fan`, `arc`, `semi` switches the geometry on; every other pattern runs the
  cage-2 code unchanged. `fan` imports `fan_geom`, `arc` `arc_geom`, `semi` `semi_geom` (each module exports the
  `fan_geom` names: `FanPattern`, `FanReadout`, `FanReadoutB`, `fan_plateaus`, `make_fan_controller`,
  `make_fan_bore_gate`).
- Refused unless `--mode shadow | closed` with ELP-B on, plan `A`, `A_b1` or `Bpp`, control sources `elp` or
  `elp,bore1`, and `--ov-outside` equal to the json's `ov_outside_required` (40) after clamping.
- `bore1_rt.bore1_gate = make_fan_bore_gate("A", --fan-bore-max-deg)` and `bore2_rt.bore2_gate =
  make_fan_bore_gate("B", --fan-bore-max-deg)`; the bore threads look the gate up at call time.
- ELP-A / ELP-B readouts are `FanReadout` / `FanReadoutB`; the controller is `make_fan_controller(Controller)`. Lines
  or positions with |delta| above `--fan-ctrl-max-deg` are read and logged but hold u.
- `semi` only: the gate log `roboter/<ses>/semi_gate_<ses>.csv` is opened next to the ctrl csv.
- Defaults `--fan-ctrl-max-deg 15`, `--fan-bore-max-deg 0`. Both sessions in `data/` ran with 90 / 90
  (`session_meta.json` → `realtime.fan`).

## Geometry and gates (from the code)

- **Kinds.** A = moving −X-ish, read by ELP-A and bore1; B = moving +X-ish, read by ELP-B and bore2. `leg` counts per
  kind in print order. `hop` rows of the segs csv are connectors and never a leg.
- **Angle.** `delta` = local tangent angle in degrees: B = path angle, A = path angle − 180, wrapped to [−180, 180).
- **Lookup.** `s` (arc length along the leg) and `e` (signed offset along the left normal) by projection; the leg of
  the camera's kind with the smallest |e| among those with −tol ≤ s ≤ L + tol (`line_tol_mm`, 1 mm).
- **ELP gate** (both ELPs, tip = RSI track at frame time − rig latency): stale ring → `stale`; no track → `no_track`;
  |z − z_cmd(x)| > 0.5 or motion over 0.1 s along t(s) < `dir_min_mm` → `z_or_dir`; no leg or x outside `x_window` →
  `leg_or_xwin`; s < `min_s_tip` → `min_s_tip`. There is no angle cap in the ELP gate.
- **ELP grid.** Rows at s_tip − d for d in 12–25 mm (0.1 mm steps) on the path itself, through the lobe joints;
  columns at e_tip + n for n in ±10.5 mm along the local normal. A row's `s_dep` = s_tip − d.
- **Bore gate** (tip = RSI track at frame time, no latency): `stale`, `no_track`, `z_or_dir` (|z − z_cmd| ≥ 0.5 or
  motion along t(s) ≤ 0.3 mm in 0.1 s), `leg_or_x` / `line_or_x` (no leg, |e| ≥ 0.5, or x outside bore1's 770–945 /
  the pattern's `x_window` for bore2), `angle` (|delta(s_tip)| > max_deg + 1e-9), else `ok`. `s_dep` = s_tip − 5.
- **Gate log** (`semi` only): one row per gated ELP or bore frame: cam, t, leg, row, s_tip, e, delta / curv / R /
  lobe at the tip and at s_obs. For bores s_obs = s_tip − 5; for the ELPs the log writes s_obs = s_tip.

## Known issues (found in the code; for the port, not fixed here)

1. **ELP binned by the tip angle.** `semi_failure_map_261103.py` bins the ELP frames of its summary by `tip_delta`.
   The ELP reads 12–25 mm behind the tip, which is d / R = 35–74 deg of turn on the R 19.375 lobes and 53–111 deg on
   the R 12.917 lobes. The per-row ELP table (`failuremap_elp_<ses>.csv`) already has the observed-angle columns
   (`obs_*` at each row's `s_dep`). The lab withdrew a conclusion drawn from the tip-binned summary. T14 bins the ELP
   by the observed angle.
2. **Bore angle cap at the tip.** The bore gate tests |delta| at s_tip, while the bore observes s_tip − 5: 15 deg of
   turn on R 19.375 and 22 deg on R 12.917. Both sessions ran with 90-deg caps, and no bore frame of either session
   was rejected as `angle` (`gate` column of `bore1_rt` / `bore2_rt`), so the cap itself is untested on real data.
   T13 reproduces the legacy rule and offers the observed position as an option.
3. `fan_geom.py` and `arc_geom.py` keep the pre-semi code paths for the `fan` and `arc` geometries; `semi` reuses
   `arc_geom`'s bore gate and controller with `semi_geom`'s lines.

## Not included

`run_cl.py` of this copy, the cage-2 ELP rigs (intrinsics and hand-eye), the bore weights, the videos, the ARC1 files,
and the pattern generator. The ELP widths in `data/` were computed live in the lab.
