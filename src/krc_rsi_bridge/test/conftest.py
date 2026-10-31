import pytest
from bridge_paths import PATTERN, free_udp_port


@pytest.fixture
def udp_port():
    return free_udp_port()


@pytest.fixture
def pattern_file():
    return str(PATTERN)
