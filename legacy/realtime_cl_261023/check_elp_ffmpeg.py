"""Which ffmpeg would run_cl --elp-codec hevc use on this PC? (261023; no camera, no RSI, ~1-3 s per candidate)

Lists the candidates in run_cl's order (--elp-ffmpeg, env RT_FFMPEG, imageio-ffmpeg, this conda env, PATH), runs the
same start-up self test as run_cl on each (24 synthetic 1920x1200 frames through the live hevc_nvenc command: exit code,
frames in the file, cv2 decode count, colour round trip) and prints the one run_cl would pick, or that run_cl would
record MJPG AVI. Exit code 0 = HEVC would be used, 1 = MJPG fallback.
python -X utf8 check_elp_ffmpeg.py [--elp-ffmpeg <ffmpeg.exe>] [--cq 18]
"""
import argparse
import json
import logging
import sys
import types

import elp_hevc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elp-ffmpeg", default=None)
    ap.add_argument("--cq", type=int, default=elp_hevc.CQ_DEFAULT)
    ap.add_argument("--res", default="1920x1200")
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    w, h = (int(x) for x in a.res.split("x"))
    print("candidates:", json.dumps(elp_hevc.ffmpeg_candidates(a.elp_ffmpeg), indent=1))
    args = types.SimpleNamespace(elp_codec="hevc", elp_ffmpeg=a.elp_ffmpeg, elp_hevc_cq=a.cq)
    rec = elp_hevc.choose(args, {"res": (w, h)})
    print(json.dumps(dict(codec=rec.codec, ffmpeg=rec.ffmpeg, version=rec.version, reason=rec.reason,
                          selftest=rec.attempts), indent=1))
    print(f"RESULT: run_cl --elp-codec hevc would record {'HEVC via ' + rec.ffmpeg if rec.codec == 'hevc' else 'MJPG AVI'}")
    sys.exit(0 if rec.codec == "hevc" else 1)


if __name__ == "__main__":
    main()
