import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from e1_paths import DATA, SES, e1_available  # noqa: E402


@pytest.fixture(scope="session")
def e1_export(tmp_path_factory):
    """E1 logs -> bag (test-only writer) -> bag_to_legacy. Returns (bag path, export root, write count)."""
    if not e1_available():
        pytest.skip("E1 data not present")
    pytest.importorskip("rosbag2_py")
    pytest.importorskip("krc_interfaces.msg")
    from e1_bag import e1_items

    from print_session.bag_to_legacy import main as export_main
    from print_session.bag_io import write_bag

    root = tmp_path_factory.mktemp("e1_roundtrip")
    bag = root / "session" / "bag"
    bag.parent.mkdir()
    n = write_bag(bag, e1_items())
    out = root / "export"
    assert export_main([str(bag), "--out", str(out), "--session", SES,
                        "--meta", str(DATA / "raw_video" / "session_meta.json")]) == 0
    return bag, out, n
