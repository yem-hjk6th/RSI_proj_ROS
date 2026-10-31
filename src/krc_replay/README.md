# krc_replay

End-to-end replay of a logged session through the ROS graph (T5, issue #10). It is the gate before shadow mode on
hardware: the nodes must give the same result as the library ports, and the real-time chain must keep the safety
invariants over a whole recorded session.

## Contents

| Module | ROS | Role |
|---|---|---|
| `session.py` | no | Loads a session folder (`data/e1_<ses>/`) as replay events: RSI packets, ELP and bore1 readouts, controller update times. Lock-step schedule (T2's golden merge), IPOC keying, `Width` row encoding. |
| `params.py` | no | Graph parameters from `config/e1.yaml`. The bridge takes its limits from the controller section and `ov_default` as the controller derives it. |
| `recording_sim.py` | no | `krc_sim`'s `KrcSim` with a per-reply record (IPOC, OvCmd, RKorr). Motion, timing and Timeout logic are unchanged. |
| `report.py` | no | Chain checks and the agreement report; linear-ramp check. |
| `log_player.py` | yes | `log_player` node: `lockstep` or `realtime` mode. |
| `latency_test.py` | no | `latency_test` CLI: `krc_sim` against a bridge on another host, and its report (`report.link_report`). |

### log_player

- `KrcState` from `roboter/rsi_data_<ses>.csv.gz`. `path_speed_mm_s` and `v_eff_mm_s` come from `krc_rsi_bridge.path_speed`, as in the bridge.
- `Width` on `/width/elp` from `roboter/readout_rows_<ses>.csv.gz` (frames with `n_used > 0`), pushed at `t_done_perf`.
- `Width` on `/width/bore1` from `raw_video/bore1_rt_<ses>.csv.gz` rows with `pushed = 1` (d = 5 mm), pushed at `t_done`.
- One `Width` per row. `s_mm` is the logged `s_dep` bit for bit. `d_mm` is chosen so that `s_mm + d_mm` (how `cl_controller` rebuilds the tip) gives the logged `s_tip`. It differs from the logged `d` by about 1e-14 mm; see `session.encode_rows`.
- `lockstep`: per logged update, everything logged at or before it, then `/cl_controller/step`. It waits for that update's `OverrideCmd` before going on. `Width.stamp` is the logged push time.
- `realtime`: `Width` only, keyed to the IPOC of each `KrcState` the bridge publishes. `Width.stamp` is the frame time on the live clock.

## Launch

```bash
ros2 launch krc_replay replay_lockstep.launch.py params_file:=config/e1.yaml \
    session_dir:=data/e1_20261022_165128 out_csv:=lockstep_ov.csv
ros2 launch krc_replay replay_realtime.launch.py params_file:=config/e1.yaml \
    session_dir:=data/e1_20261022_165128 start_row:=3000 n_packets:=14750
```

Paths are relative to the repository root, or absolute. `n_packets:=0` replays to the end of the session.
The real-time launch uses `krc_sim`'s own `sim_node` in replay mode on loopback UDP. It ends when the simulator ends.

## Latency test (T9)

`krc_sim` (pattern mode, its own process) against a `krc_rsi_bridge` on another host, for a given time. The bridge
side runs the bridge alone, in closed mode, with the session parameter file:

```bash
ros2 launch krc_replay latency_bridge.launch.py params_file:=config/e1.yaml
```

The simulator side:

```bash
ros2 run krc_replay latency_test --host <BRIDGE_IP> --port 59152 --duration 60 --params-file config/e1.yaml
```

It prints packets sent, late replies and their longest run, Timeout yes / no, the round trip of the on-time replies
(p50 / p99 / max), replies with nonzero RKorr, OvCmd outside the window (and replies without OvCmd), and PASS / FAIL.
Exit code 0 = PASS, 1 = FAIL, 2 = the bridge did not answer within `--wait` s. The setup on the lab laptop (WSL 2,
firewall) and the expected readings are in `docs/deploy_wsl.md` §4.

## Tests

| File | Test (T5) | ROS |
|---|---|---|
| `test_session.py` | core: row encoding, lock-step schedule, node-layer conversion without ROS, slice choice | no |
| `test_lockstep_ros.py` | 1: lock-step equivalence, 1719 updates, `value_pct` and `source` equal the legacy fixture | yes |
| `test_elp_node_ros.py` | 2: `width_readout_elp` node on the 40 frames equals the golden rows | yes |
| `test_latency.py` | T9: the latency report; the CLI against the bridge's reply process (no ROS); the two commands above on loopback for 15 s, report printed | partly |
| `test_realtime_chain.py` | 3: real-time chain on a 59 s slice; 4: whole session (`KRC_REPLAY_FULL=1`, manual workflow `replay-full`); 5: heartbeat loss | yes |

**Slice (test 3).** `rsi_data` rows 3000–17750: 12.0 s to 71.0 s after the first packet, 59 s. It contains:
- the lead-in;
- the print-window opening at 22.0 s;
- the first controller publish at 22.3 s;
- all of leg 1 (start 54.5 s, end 69.2 s).

E1's first leg start is 32 s after the window opening, so a 60 s slice cannot also contain a second leg.

**Heartbeat loss (test 5).** Same start. The controller is stopped at the logged IPOC of 65.5 s, on leg 1, where the logged OvCmd is 34–39 (away from `ov_default` 40). The run goes on to 70.0 s.

The chain tests read the bridge's OvCmd, source and window state per IPOC from `/krc/reply_record` (`ReplyRecord`).

The ROS tests run on their own ROS domain (50–99, set in `test/conftest.py`; `KRC_REPLAY_ROS_DOMAIN_ID` overrides it), so they do not cross-talk with other packages' tests that `colcon test` runs in parallel.

Set `KRC_REPLAY_REPORT_DIR` to write the reports (`.json`, `.txt`) of tests 3–5.

```bash
python3 -m pytest src/krc_replay/test            # with a sourced workspace for the ROS tests
```
