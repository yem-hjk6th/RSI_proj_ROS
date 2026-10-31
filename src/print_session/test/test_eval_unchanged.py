"""Acceptance test 2: legacy eval_v3_print.py runs unchanged on the exported logs and gives the same numbers.

(a) The legacy eval on the original E1 logs reproduces roboter/eval_v3_20261022_165128.json. Keys that cannot be
    reproduced from this repository are listed in NOT_REPRODUCIBLE with the reason; everything else must be equal
    (floats to 1e-12 relative: the stored file was written with an older pandas).
(b) The legacy eval on the exported logs gives exactly the result of (a), key for key.
The bore1 shadow logs (raw_video bore1_rt_ / fusion_<ses>.csv) are not produced from the bag (bore1 is out of scope);
both runs read the original copies.
"""
import gzip
import json
import math
import shutil
import subprocess
import sys

import pytest

from e1_paths import DATA, EVAL, LEGACY_PATTERN, SES, e1_available

NOT_REPRODUCIBLE = {
    "b_elp": "needs the caliper csv (--caliper), which is not in this repository",
    "b_bore1": "needs the caliper csv",
    "caliper_points": "needs the caliper csv",
    "caliper_share_in_T": "needs the caliper csv",
    "controlled_by": "added by fix 5 (line A 261023) of the eval in legacy/; the stored json is from the 261022 eval",
    "pattern": "the path given with --pattern (the stored run used a Windows relative path)",
}

pytestmark = pytest.mark.skipif(not (e1_available() and EVAL.exists()), reason="E1 data or legacy eval not present")


def _layout(root, roboter_src=None):
    """<root>/roboter/<ses>/*.csv (from roboter_src, else the gzipped originals) + raw_video/<ses>/ originals."""
    rd, vd = root / "roboter" / SES, root / "raw_video" / SES
    rd.mkdir(parents=True), vd.mkdir(parents=True)
    for name in ("rsi_data", "ov_cmd", "ctrl", "readout", "readout_rows"):
        if roboter_src is None:
            with gzip.open(DATA / "roboter" / f"{name}_{SES}.csv.gz", "rb") as f, open(rd / f"{name}_{SES}.csv",
                                                                                     "wb") as g:
                shutil.copyfileobj(f, g)
        else:
            shutil.copyfile(roboter_src / f"{name}_{SES}.csv", rd / f"{name}_{SES}.csv")
    for name in ("bore1_rt", "fusion"):
        with gzip.open(DATA / "raw_video" / f"{name}_{SES}.csv.gz", "rb") as f, open(vd / f"{name}_{SES}.csv",
                                                                                   "wb") as g:
            shutil.copyfileobj(f, g)
    shutil.copyfile(DATA / "raw_video" / "session_meta.json", vd / "session_meta.json")
    return root


def _eval(root):
    out = root / "eval"
    r = subprocess.run([sys.executable, str(EVAL), "--data", str(root), "--session", SES, "--pattern",
                        str(LEGACY_PATTERN), "--out", str(out)], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr[-3000:]
    return json.loads((out / f"eval_v3_{SES}.json").read_text(encoding="utf-8"))


def _eq(a, b, rel=0.0):
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_eq(a[k], b[k], rel) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_eq(x, y, rel) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
        return a == b or abs(a - b) <= rel * max(abs(a), abs(b))
    return a == b


@pytest.fixture(scope="module")
def eval_orig(tmp_path_factory):
    return _eval(_layout(tmp_path_factory.mktemp("eval_orig")))


def test_a_legacy_eval_reproduces_the_stored_result(eval_orig):
    stored = json.loads((DATA / "roboter" / f"eval_v3_{SES}.json").read_text(encoding="utf-8"))
    keys = (set(stored) | set(eval_orig)) - set(NOT_REPRODUCIBLE)
    bad = [k for k in sorted(keys) if not _eq(eval_orig.get(k), stored.get(k), rel=1e-12)]
    assert not bad, {k: (eval_orig.get(k), stored.get(k)) for k in bad}
    assert eval_orig["success"] is True and eval_orig["branch"] == stored["branch"]


def test_b_same_numbers_on_the_exported_logs(eval_orig, e1_export, tmp_path):
    _, out, _ = e1_export
    got = _eval(_layout(tmp_path / "exp", out / "roboter" / SES))
    bad = [k for k in sorted(set(got) | set(eval_orig)) if not _eq(got.get(k), eval_orig.get(k))]
    assert not bad, {k: (got.get(k), eval_orig.get(k)) for k in bad}
