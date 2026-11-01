# Legacy pipeline (reference only)

**What this is**
- `realtime_cl_261023/` is the Windows/Python closed-loop pipeline as it ran in the lab on 2026-10-22.
- `pipeline_cage1012/rsi_listener.py` is the one file of the pipeline folder that `rsi_proc.py` imports (`build_zero_reply`, `parse_rsi_xml`). It is the copy from the 2026-10-14 package. The rest of that folder is not included, and its `config` import is not satisfied here.
- `krc_config/` holds the controller-side files it ran with:
  - the RSI configuration (`rsi/`); the PC address is replaced by `<PC_IP>`:
    - `RSI_OV4.rsi*` and `RSI_EthernetConfig_OV4.xml`: the configuration the print program loads (`RSI_CREATE("RSI_OV4.rsi")`). OvCmd in each reply goes through `LIMIT1` (1..100) to `$OV_PRO`. These are the copies from the 2026-10-20 package; the controller's own copy was not read back.
    - `RSI_MIN.rsi*` and `RSI_EthernetConfig.xml`: an earlier configuration without OvCmd (RKorr only). The lab's print programs no longer load it.
  - `sps.sub`;
  - one KRL print program (`krl/`).

**Shadow mode does not fit the lab's print programs**
- In shadow mode the reply carries no `<OvCmd>`. `run_cl.py` (lines 4 and 7) records that shadow mode on `RSI_OV4` means 1 % override. The configuration explains why: with no OvCmd the input is 0, and `LIMIT1` raises it to 1.
- The lab's print programs load `RSI_OV4`. A run that computes and logs everything without acting is `--mode closed` with plan `Bpp` (constant OvCmd), as for the open-loop prints.
- The ROS port's shadow mode sends the same bytes, so the same applies to it.

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
