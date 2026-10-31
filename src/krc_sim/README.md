# krc_sim

Fake KUKA KR C4 RSI client, so the bridge can be tested without hardware.

- Sends one packet every `period_s` (4 ms). The format is the legacy fake controller's (`legacy/realtime_cl_261023/harness_run_cl.py`), following the SEND section of `legacy/krc_config/rsi/RSI_EthernetConfig.xml`.
- `pattern` mode: the TCP moves along a pattern json (`patterns/`, copied from `legacy/realtime_cl_261023/pattern/`) at `v100_mm_s` × override / 100.
  - The override is the last on-time reply's `OvCmd`, limited to 1..100.
  - A reply without `OvCmd` gives 1 %, as noted in the KRL program header.
- `replay` mode: streams a recorded `rsi_data_*.csv(.gz)`.
- Counts late replies. A reply is late when it has not arrived one period after its packet; a reply that comes later carries an old IPOC and is discarded.
  - The counter counts consecutive late replies and is sent in `<Delay D=".."/>`.
  - When it reaches `timeout_packets` (100, the ETHERNET Timeout), the simulator stops like RSI.

```bash
ros2 run krc_sim sim_node --ros-args -p pattern_file:=<PATH_TO>/pattern_usr_ELP_R1v30_CLp_261022.json
```

The core (`krc_sim/sim.py`, `path.py`, `packet.py`) has no ROS imports. `SimProcess` runs it in its own process.
