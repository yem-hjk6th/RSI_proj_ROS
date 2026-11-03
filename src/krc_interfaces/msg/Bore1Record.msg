# One bore1 frame processed by width_readout_bore1: a row of the legacy bore1_rt_<ses>.csv (B1_COLS of legacy
# bore1/bore1_rt.py, same names, same order; t_frame is the stamp). Published on /width/bore1/record for every frame
# the node processes, also when the gate rejects it; frames that arrive while the node is busy are skipped and have no
# record (as in the legacy thread). Values are unrounded. Where the legacy log has an empty cell (gate != ok) a float
# is NaN, leg is -1 and an int8 tri-state is -1.
builtin_interfaces/Time stamp   # frame time, the time the gate used (same clock and value as Width.stamp of the frame)
uint64 frame                    # frames processed by the node, 1, 2, ... (legacy: the camera's frame index)
float64 t_done                  # end of the frame's processing on CLOCK_MONOTONIC, s (same clock as ReplyRecord.pc_ts_perf)
string gate                     # ok | stale | no_track | z_or_dir | leg_or_x
int32 leg                       # 0-based leg as in the pattern json; -1 = gate != ok
float64 s_tip                   # mm along the leg: nozzle position at the frame time
float64 s_dep                   # mm along the leg where the material under the borescope was laid: s_tip - 5
float64 x                       # mm, RIst x at the frame time (interpolated on the RSI track)
float64 y                       # mm
float64 z                       # mm
float64 w                       # mm, bead width: mean of the three seeds (the deploy value)
float64 w_s0                    # mm, seed 0
float64 w_s1                    # mm, seed 1
float64 w_s2                    # mm, seed 2
float64 sd                      # mm, standard deviation of the three seeds (ddof 0)
int8 q_ok                       # frame quality inside the training limits: 1 | 0 | -1 = empty
float64 bright                  # mean grey level of the 288 x 288 crop
float64 sat                     # fraction of saturated pixels (grey >= 250)
float64 sharp                   # variance of the Laplacian
float64 diff                    # mean |crop - previous crop| (grey); NaN on the first frame
float64 ms                      # head compute time (crop, quality, three models, sync), ms
int8 pushed                     # the row was published on /width/bore1 with valid = true: 1 | 0 | -1 = empty
