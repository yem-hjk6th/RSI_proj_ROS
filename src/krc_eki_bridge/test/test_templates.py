"""Controller-side templates: NOT RUN headers, no motion, $OV_PRO written only under the AUT / EXT guard."""
import re
from pathlib import Path

import pytest

TPL = Path(__file__).resolve().parents[1] / "krc_templates"
FILES = sorted(p for p in TPL.iterdir() if p.is_file())


def test_templates_present():
    names = {p.name for p in FILES}
    assert {"KrcEki.xml", "sps_eki_override.sub", "eki_send_state.src"} <= names


@pytest.mark.parametrize("path", FILES, ids=lambda p: p.name)
def test_not_run_header(path):
    head = "\n".join(path.read_text(encoding="utf-8").splitlines()[:5])
    assert "NOT RUN" in head


@pytest.mark.parametrize("name", ["sps_eki_override.sub", "eki_send_state.src"])
def test_no_motion_in_krl(name):
    code = "\n".join(l.split(";")[0] for l in (TPL / name).read_text(encoding="utf-8").splitlines())
    for kw in ("PTP", "LIN", "CIRC", "SPL", "SLIN", "SPTP", "RSI_", "RKorr", "$POS_RET"):
        assert not re.search(rf"\b{re.escape(kw)}", code, re.IGNORECASE), kw


def test_ov_pro_written_only_in_aut_ext():
    lines = [l.split(";")[0].rstrip() for l in (TPL / "sps_eki_override.sub").read_text(encoding="utf-8").splitlines()]
    guard_depth = None
    depth = 0
    writes = 0
    for line in lines:
        s = line.strip().upper()
        if s.startswith("IF ") and "$MODE_OP == #AUT" in s and "$MODE_OP == #EX" in s:
            guard_depth = depth
        if s.startswith("IF "):
            depth += 1
        if s.startswith("ENDIF"):
            depth -= 1
            if guard_depth is not None and depth == guard_depth:
                guard_depth = None
        if re.match(r"\$OV_PRO\s*=", s):
            writes += 1
            assert guard_depth is not None, line
    assert writes == 2   # the commanded value and the safe default


def test_eki_xml_receives_only_override_and_seq():
    xml = (TPL / "KrcEki.xml").read_text(encoding="utf-8")
    recv = xml.split("<RECEIVE>")[1].split("</RECEIVE>")[0]
    tags = re.findall(r'Tag="([^"]+)"', recv)
    assert tags == ["Ext/Ov", "Ext/Seq"]
    assert "<PROTOCOL>TCP</PROTOCOL>" in xml
    assert "<PORT><EKI_PORT></PORT>" in xml
