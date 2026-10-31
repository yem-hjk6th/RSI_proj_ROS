# krc_rsi_bridge

RSI UDP server for the KR C4 (route A in `docs/DESIGN.md`). Ported from `legacy/realtime_cl_261023/rsi_proc.py`, `rsi_ov_listener.py` and `rsi_ring.py`.

**Status:** tested against `krc_sim` only. Not run against a real controller yet.

## Structure

| Module | Role | ROS |
|---|---|---|
| `rsi_xml.py` | Packet parser, reply builder (RKorr is a constant zero string) | no |
| `window.py` | Pattern geometry and print-window state machine | no |
| `decider.py` | Per-packet OvCmd: outside / forced / step / control / watchdog, ramp, clamp, speed floor | no |
| `shared.py` | Lock-free shared values and the packet ring (reply loop → ROS) | no |
| `server.py` | The reply loop in its own process (`spawn`), and its parent-side handle | no |
| `path_speed.py` | Chord path speed and `v_eff` for `KrcState` | no |
| `bridge_node.py` | Thin ROS 2 node: `/krc/state` out, `/krc/override_cmd` in | yes |

The reply loop does only this per packet: receive, parse, decide, reply. Then it writes the packet into a shared-memory ring. It takes no locks, does no logging and never waits on the parent. The ROS side writes the controller's value and heartbeat into shared scalars and reads the ring on a timer.

## Topics

- `/krc/state` (`krc_interfaces/KrcState`): one message per RSI packet. `stamp` is the kernel receive time of the packet. `late_replies` counts replies sent more than `deadline_s` after the packet arrived.
- `/krc/override_cmd` (`krc_interfaces/OverrideCmd`):
  - A message whose `seq` differs from the previous one is a heartbeat and a new value.
  - `source: default` is the controller default. Any other source counts as control.
  - `source: forced` sets the forced latch. The latch is never cleared until restart.

## Run with the simulator

```bash
ros2 launch krc_rsi_bridge sim_loop.launch.py
```

Parameters: see `config/example.yaml` (network values are placeholders).

## Tests

```bash
python3 -m pytest src/krc_rsi_bridge/test src/krc_sim/test
```

- `test/fixtures/` holds golden outputs of the legacy reply process. Regenerate them with `test/tools/make_golden.py`; it needs numpy, scipy and OpenCV for the legacy imports.
- `test_stall_ros.py` needs a sourced ROS 2 workspace. It is skipped otherwise.
