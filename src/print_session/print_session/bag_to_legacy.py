"""Export a print-session bag to the legacy CSV logs, so that legacy eval_v3_print.py runs on it unchanged.

    ros2 run print_session bag_to_legacy <session_dir>/bag --out <out_dir>
    python3 legacy/realtime_cl_261023/eval_v3_print.py --data <out_dir> --session <ses> --pattern <pattern json>

Writes <out>/roboter/<ses>/{rsi_data,ov_cmd,ctrl,readout,readout_rows}_<ses>.csv and copies session_meta.json to
<out>/raw_video/<ses>/. The session name comes from --session, else session_meta.json, else the bag's parent folder.
"""
import argparse
import json
import sys
from pathlib import Path

from . import topics as T
from .export import export
from .legacy_cols import NOT_CARRIED


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="bag folder (as written by the recorder)")
    ap.add_argument("--out", required=True, help="output root (gets roboter/<ses>/ and raw_video/<ses>/)")
    ap.add_argument("--session", default=None, help="session name used in the file names")
    ap.add_argument("--meta", default=None, help="session_meta.json (default: next to the bag folder)")
    ap.add_argument("--elp-topic", default=T.WIDTH_ELP, help="Width topic of the ELP readout")
    a = ap.parse_args(argv)
    bag = Path(a.bag)
    meta = Path(a.meta) if a.meta else bag.parent / "session_meta.json"
    ses = a.session
    if ses is None and meta.exists():
        ses = json.loads(meta.read_text(encoding="utf-8")).get("session_ts")
    ses = ses or bag.resolve().parent.name
    from .bag_io import read_bag
    msgs = read_bag(bag, [T.STATE, T.REPLY, T.CTRL, a.elp_topic])
    counts = export(msgs, a.out, ses, meta if meta.exists() else None, a.elp_topic)
    for name, n in counts.items():
        print(f"{n:8d} rows  {Path(a.out) / 'roboter' / ses / name}")
    for f, cols in NOT_CARRIED.items():
        print(f"not carried by the topics ({f}): " + ", ".join(k for k in cols if not k.startswith("_")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
