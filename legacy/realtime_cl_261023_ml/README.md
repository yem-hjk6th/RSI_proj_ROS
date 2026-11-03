# realtime_cl_261023_ml (reference only)

The multi-layer (walls) variant of `../realtime_cl_261023/`, prepared for the wall programs `usr_ELP_W4L28_*`.

- Only the files that differ from `../realtime_cl_261023/` are here. Every other module of the lab folder is byte-for-byte
  the same as in `../realtime_cl_261023/` (checked 2026-11-02 against the lab copy).
- Same publication rewrites as `../README.md` (identifiers, comments).

| File | Lines changed | What changes |
|---|---|---|
| `rt_readout.py` | 70 | `Pattern` reads an optional `z_off` column of the segs csv (planned Z above the layer-1 plane, per segment). With it, the gate takes the nearest planned layer and that layer's leg, and the readout plane is lifted by the leg's offset. Without it, the single-layer code path is unchanged. |
| `rt_controller.py` | 44 | the sources filter and the wall stop rule (`sat_alarm`) |
| `run_cl.py` | 22 | `--control-sources` (which sources may steer; walls: `elp`, because the bore1 head is a layer-1 model and stays in shadow); `--sat-stop-s` (on a seen leg of layer >= 2, OvCmd at `ov_max` from the feedback for that many seconds in a row -> forced open loop, latched; default 2.0 s on wall patterns, off otherwise) |
| `rsi_ring.py` | 10 | `veff_at_s(..., pass_only)`: a -X pass can start right after the same wall's +X pass one layer lower, so only the packets of the current pass are used |
| `rt_law_v3.py` | 6 | `LawCfg.sources`: drop the sources that may not steer from the source order |

`pattern/` holds the three wall patterns (`W4L28_CL`, `W4L28_OLg`, `W4L28_OLs`) with their segs csv.

No wall session log is in this repository.
