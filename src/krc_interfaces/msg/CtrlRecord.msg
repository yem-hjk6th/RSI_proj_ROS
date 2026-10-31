# One controller update: a row of the legacy ctrl_<ses>.csv (CTRL_COLS of legacy run_cl.py, same names, same order).
# Published by cl_controller on /controller/ctrl_record after every law update (every t_u = 0.25 s while it runs).
# Values are unrounded. A float is NaN where the legacy log has an empty cell; an int8 tri-state is -1 for empty.
builtin_interfaces/Time stamp   # update time on the node clock (legacy t_wall)
float64 t_perf                  # update time on CLOCK_MONOTONIC, s (legacy t_perf; same clock as ReplyRecord.pc_ts_perf)
uint32 n_upd                    # updates so far
string win                      # controller print window: before | inside | after
int32 leg                       # 0-based leg as in the pattern json; -1 = not on a print leg
bool on_a
float64 s_tip                   # mm along the leg
int32 n_bins
float64 veff                    # mm/s
float64 w_est                   # mm
float64 rhat
float64 rhat_f                  # legacy column name; holds u, as the legacy log does
float64 logk
float64 slow
bool have_est
float64 k
float64 ov_raw
int32 ov_round
int32 ov_pub                    # value handed to the bridge (ov_default while src = outside)
string src                      # outside | control | default
float64 rows_age_s
bool forced
string plan                     # A | A_b1 | B | Bp | Bpp
string state                    # elp | bore1 | model, empty before the first update inside the window
string reason
string gate
int8 valid_elp                  # 1 | 0 | -1 = empty
string why_elp
float64 rhat_elp
float64 lag_elp
int8 valid_b1                   # 1 | 0 | -1 = empty
string why_b1
float64 rhat_b1
float64 lag_b1
float64 e
float64 du
float64 u
float64 lnk_ff
float64 v_tip                   # mm/s
uint32 n_corr
uint32 n_switch
float64 w_target                # mm
float64 lnk_tgt
float64 nobed_s
int8 hold_bed                   # 1 | 0 | -1 = empty
float64 lnk_ol
float64 du_blind
float64 t_blind
