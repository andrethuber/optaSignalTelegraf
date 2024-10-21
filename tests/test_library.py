import library
import pytest


@pytest.mark.parametrize("value", range(128))
def test_ip_range_is_limited(value):
    assert 100 <= library.getIpsFromDip(value, 1) <= 227
    assert 100 <= library.getIpsFromDip(value, 0) <= 227


@pytest.mark.parametrize("value", range(128, 256))
def test_ip_is_null_if_out_of_range(value):
    assert library.getIpsFromDip(value, 1) == 0
    assert library.getIpsFromDip(value, 0) == 0


@pytest.mark.parametrize("value", range(128))
def test_remote_ip_is_always_one_away(value):
    assert abs(library.getIpsFromDip(value, 1) - library.getIpsFromDip(value, 0)) == 1


def test_get_ips_from_example_dip():
    assert library.getIpsFromDip(0, 1) == 100
    assert library.getIpsFromDip(1, 1) == 101
    assert library.getIpsFromDip(2, 1) == 102
    assert library.getIpsFromDip(3, 1) == 103
    assert library.getIpsFromDip(4, 1) == 104

    assert library.getIpsFromDip(0, 0) == 101
    assert library.getIpsFromDip(1, 0) == 100
    assert library.getIpsFromDip(2, 0) == 103
    assert library.getIpsFromDip(3, 0) == 102
    assert library.getIpsFromDip(4, 0) == 105
