# Legacy pipeline (reference only)

**What this is**
- `realtime_cl_261023/` is the Windows/Python closed-loop pipeline as it ran in the lab on 2026-10-22.
- `realtime_cl_261023_ml/` holds only the five modules and the wall patterns that differ in the multi-layer (walls) variant of the same pipeline; see its `README.md`.
- `pipeline_cage1012/` holds the files of the recorder folder that `run_cl.py` imports:
  - `rsi_listener.py`, which `rsi_proc.py` imports (`build_zero_reply`, `parse_rsi_xml`); the copy from the 2026-10-14 package;
  - `elp.py` (ELP capture and MJPG writer), `borescope.py` (borescope capture and mp4v writer), `config.py` and `pipeline.py`, added 2026-11-02 as the reference for camera capture and recording (task T10). The rest of that folder is not included.
- `krc_config/` holds the controller-side files it ran with:
  - the RSI configuration (`rsi/`); the PC address is replaced by `<PC_IP>`:
    - `RSI_OV4.rsi*` and `RSI_EthernetConfig_OV4.xml`: the configuration the print program loads (`RSI_CREATE("RSI_OV4.rsi")`). OvCmd in each reply goes through `LIMIT1` (1..100) to `$OV_PRO`. These are the copies from the 2026-10-20 package. The lab states (2026-11-02) that the controller still runs this last recorded change; the controller's copy was not read back byte for byte.
    - `RSI_MIN.rsi*` and `RSI_EthernetConfig.xml`: an earlier configuration without OvCmd (RKorr only). The lab's print programs no longer load it.
  - `sps.sub`;
  - KRL programs (`krl/`): one print program, and `usr_OV_step_261020.src`, the air step test program (no extrusion, `RSI_OV4`) that `run_cl.py --mode step --window-ignore-z` ran with the pattern `pattern_usr_ELP_R1v_261016.json` on 2026-10-20. Its header still says `NOT RUN ON HARDWARE YET`; it ran in two lab sessions that day.

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
