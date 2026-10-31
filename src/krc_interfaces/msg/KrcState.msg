# One RSI packet from the KR C4, as received by the bridge.
builtin_interfaces/Time stamp   # PC receive time
uint64 ipoc                     # controller timestamp, echoed in the reply
float64[6] rist                 # X, Y, Z [mm], A, B, C [deg], as sent by the controller
float64 path_speed_mm_s         # chord speed over the last 25 packets
float64 v_eff_mm_s              # first-order lag of path speed (tau 0.28 s): speed the bead is laid at
int32 override_pct              # override reported by the controller
float64 vel_act_mm_s            # $VEL_ACT as sent (1 mm/s resolution)
int32 rpm_ext                   # extruder motor rpm command from sps.sub
uint32 late_replies             # replies the bridge sent after the 4 ms deadline, cumulative
