# One change of the controller's effective source: a row of the legacy switch_<ses>.csv (SWITCH_COLS of legacy
# realtime_cl_cage2_semi_261103/run_cl.py, same order; "from" / "to" are from_label / to_label here). Published by
# cl_controller on /controller/switch_record. Values are unrounded; NaN where the legacy log has an empty cell.
builtin_interfaces/Time stamp   # update time on the node clock (legacy t_wall)
float64 t_perf                  # the same instant on CLOCK_MONOTONIC, s (legacy t_perf)
string from_label               # none | outside | hold | elpA | bore1 | model_A | elpB | bore2 | model_B
string to_label
int32 leg
string kind                     # A | B | -
string dir                      # -X | +X | 0
float64 vx                      # mm/s
float64 x                       # mm, RIst at the update
float64 y
float64 z
float64 s                       # mm along the line (A: x_start - x; B: x - first x of the B line)
float64 u                       # u of the law that holds it
int32 ov_cmd                    # the controller's OvCmd at the switch
string why
