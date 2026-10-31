# One RSI packet as the bridge answered it: a row of the legacy ov_cmd_<ses>.csv (pc_ts_perf, ipoc, ov_cmd, source),
# plus the packet fields of rsi_data_<ses>.csv that KrcState does not carry. Published by krc_rsi_bridge on
# /krc/reply_record from its parent side (packet ring), one per packet, next to the KrcState of the same packet.
builtin_interfaces/Time stamp   # PC receive time; the same value as KrcState.stamp of this packet
float64 pc_ts_perf              # receive time on CLOCK_MONOTONIC (time.perf_counter on Linux), s
uint64 ipoc                     # controller timestamp of the packet (matches KrcState.ipoc)
int32 ov_cmd                    # OvCmd decided for this packet, after the clamp to [ov_min, ov_max]
string source                   # outside | forced | step | default | control | watchdog | fallback
bool ov_sent                    # true when the reply carried ov_cmd (closed, step); false in shadow (zero reply)
int32 delay                     # <Delay D="..."> of the packet (late replies counted by the controller)
float64 reply_latency_s         # packet arrival (kernel timestamp) -> reply sent, s
string win                      # print window state of the bridge: before | inside | after
