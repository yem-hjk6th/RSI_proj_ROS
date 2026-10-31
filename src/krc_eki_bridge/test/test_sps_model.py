"""Python model of the sps.sub override snippet (mirrors krc_templates/sps_eki_override.sub)."""
from krc_eki_bridge.fake_eki import MODE_AUT, MODE_EXT, MODE_T1, MODE_T2, SpsOverrideModel


def test_writes_only_in_aut_ext_and_holds_otherwise():
    m = SpsOverrideModel(ov_safe=40, loss_timeout_s=0.5, ov_init=100)
    assert m.step(0.0, MODE_T1, True, (60, 1)) == 100
    assert m.step(0.1, MODE_T2, True, (65, 2)) == 100
    assert m.step(0.2, MODE_AUT, True, (70, 3)) == 70
    assert m.step(0.3, MODE_EXT, True, (75, 4)) == 75
    assert m.step(0.4, MODE_T1, True, (20, 5)) == 75   # hold the last value


def test_falls_back_on_loss():
    m = SpsOverrideModel(ov_safe=40, loss_timeout_s=0.5)
    m.step(0.0, MODE_AUT, True, (80, 1))
    assert m.step(0.4, MODE_AUT, True, (80, 1)) == 80   # same Seq, still inside the timeout
    assert m.step(0.6, MODE_AUT, True, (80, 1)) == 40   # stale Seq
    m.step(0.7, MODE_AUT, True, (90, 2))
    assert m.step(0.71, MODE_AUT, False, None) == 40    # connection lost


def test_clamps_received_value():
    m = SpsOverrideModel()
    assert m.step(0.0, MODE_AUT, True, (150, 1)) == 100
    assert m.step(0.1, MODE_AUT, True, (-5, 2)) == 0
