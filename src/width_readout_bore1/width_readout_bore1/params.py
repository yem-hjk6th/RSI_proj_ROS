"""Parameters of the width_readout_bore1 node, with their defaults (no ROS imports: print_session's start-up checks
read them too), and ``check``: the refusals of a parameter set.

Cage 2 (T15): the same node reads bore2 with ``camera: bore2`` (run it as a second node, name width_readout_bore2,
topics /bore2/image_raw -> /width/bore2, /width/bore2/record). Refusals of the legacy run_cl.py
(legacy/realtime_cl_cage2_semi_261103) for the bore flags: bore2 without a quality limits file (run_cl.py 1125), bore1
with other weights than the E1 head and no quality limits file (1120), a quality file that does not exist (1123), a
width offset above 0.30 mm (1127). The head itself refuses weights that fail the sha256 check (1365: no head, no bore2
control).
"""
import math
from pathlib import Path

FRAME_TIMES = ("receive", "header")
ANGLE_AT = ("tip", "obs")
STEP_TOPIC = "/width_readout_bore1/step"

PARAMS = dict(
    camera="bore1",                     # bore1 | bore2 (cage 2: the +X lines): gate, crop, Width.source
    image_topic="/bore1/image_raw",
    state_topic="/krc/state",
    width_topic="/width/bore1",
    record_topic="/width/bore1/record",
    pattern_json="",                    # empty = width_readout_elp's packaged R1v30_CLp pattern; a path or a file
                                        # name in path_geometry's patterns/ (FAN1, ARC2)
    root="",
    head="torch",                       # torch | replay
    weight_dir="",                      # torch: folder with cnxt_t_deploy_s0.pt .. s2.pt
    quality_json="",                    # empty = the packaged copy of the legacy quality_limits.json (bore1 with the
                                        # E1 weights only; bore2 and the cage-2 bore1 weights need it set)
    weights="",                         # weight set of the sha256 check: e1 | cage2; empty = bore1 e1, bore2 cage2
    w_offset=0.0,                       # mm added to every published width (run_cl --bore1/2-offset-mm), |x| <= 0.30
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

WIDTH_TOPICS = {"bore1": ("/width/bore1", "/width/bore1/record"), "bore2": ("/width/bore2", "/width/bore2/record")}
OFFSET_MAX_MM = 0.30


def check(p: dict) -> None:
    """Raise ValueError with the reason when the node must not start with the parameters ``p`` (PARAMS keys)."""
    from .config import DEFAULT_WEIGHTS, sha_json_for
    cam = p["camera"]
    if cam not in WIDTH_TOPICS:
        raise ValueError(f"camera {cam!r}: bore1 | bore2")
    sha_json_for(cam, p["weights"])                              # the weight set exists for the camera
    off = float(p["w_offset"])
    if not (math.isfinite(off) and abs(off) <= OFFSET_MAX_MM):
        raise ValueError(f"w_offset {off}: need |x| <= {OFFSET_MAX_MM:.2f} mm (run_cl.py 1127)")
    q = str(p["quality_json"] or "")
    if q and not Path(q).is_file():
        raise ValueError(f"quality_json {q}: file not found (run_cl.py 1123)")
    if cam == "bore2" and not q:
        raise ValueError("camera bore2 needs quality_json (the bore2 quality limits; run_cl.py 1125)")
    if cam == "bore1" and (p["weights"] or DEFAULT_WEIGHTS["bore1"]) != DEFAULT_WEIGHTS["bore1"] and not q:
        raise ValueError(f"camera bore1 with weights {p['weights']!r} needs quality_json: the packaged "
                         f"quality_limits.json belongs to the frozen E1 head (run_cl.py 1120)")
    other = WIDTH_TOPICS["bore2" if cam == "bore1" else "bore1"]
    for k in ("width_topic", "record_topic"):
        if p[k] in other:
            raise ValueError(f"camera {cam} publishing on {p[k]}: that topic carries the other borescope's rows")
