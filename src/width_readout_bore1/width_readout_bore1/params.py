"""Parameters of the width_readout_bore1 node, with their defaults (no ROS imports: print_session's start-up checks
read them too)."""
import math

FRAME_TIMES = ("receive", "header")
ANGLE_AT = ("tip", "obs")
STEP_TOPIC = "/width_readout_bore1/step"

PARAMS = dict(
    camera="bore1",                     # Width.source
    image_topic="/bore1/image_raw",
    state_topic="/krc/state",
    width_topic="/width/bore1",
    record_topic="/width/bore1/record",
    pattern_json="",                    # empty = width_readout_elp's packaged R1v30_CLp pattern; a path or a file
                                        # name in path_geometry's patterns/ (FAN1, ARC2)
    root="",
    head="torch",                       # torch | replay
    weight_dir="",                      # torch: folder with cnxt_t_deploy_s0.pt .. s2.pt
    quality_json="",                    # torch: empty = the packaged copy of the legacy quality_limits.json
    device="cuda",
    mem_frac=0.0,                       # torch: torch.cuda.set_per_process_memory_fraction; 0 = not set
    warmup=5,                           # torch: dummy frames at start (legacy load_head)
    replay_csv="",                      # replay: bore1_rt_<ses>.csv(.gz)
    replay_tol_s=1e-6,                  # replay: frame time match tolerance
    frame_time="receive",               # receive | header
    header_offset_s=0.0,                # header: added to header.stamp
    wait_s=0.06,                        # wait for an RSI packet at or after the frame time (legacy Bore1Thread)
    drop_bad_q=True,                    # q_ok False -> valid false (legacy default)
    sd_max=math.inf,                    # seed sd above this -> valid false (legacy default: no limit)
    session_meta="",                    # session_meta.json to write the bore1_readout block into; empty = none
    bore_max_deg=0.0,                   # fan / semi: angle cap |delta| in deg (run_cl.py --fan-bore-max-deg: 0)
    bore_angle_at="tip",                # fan / semi: tip (legacy) | obs (s_tip - 5)
    gate_log_dir="",                    # semi: folder of semi_gate_<camera>.csv; empty = the session_meta folder, or
                                        # no log without session_meta
    lockstep=False,
)
