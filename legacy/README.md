# Legacy pipeline (reference only)

**What this is**
- `realtime_cl_261023/` is the Windows/Python closed-loop pipeline as it ran in the lab on 2026-10-22.
- `krc_config/` holds the controller-side files it ran with:
  - the RSI configuration (`rsi/`); the PC address is replaced by `<PC_IP>`;
  - `sps.sub`;
  - one KRL print program (`krl/`).

**Changes from the lab copy**
- Absolute paths, identifiers and comments were rewritten for publication. The `usr_` prefix replaces a personal program prefix.
- So the scripts do not run unmodified from this folder.
- The logic is unchanged.

**Main entry points**
- `run_cl.py`: session orchestration.
- `rsi_proc.py`: the RSI UDP server in its own process.
- `rt_readout.py`: the ELP width readout.
- `rt_controller.py` and `rt_law_v3.py`: the control law.
- `eval_v3_print.py`: run evaluation.
