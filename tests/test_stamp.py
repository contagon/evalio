from evalio.types import Stamp


def test_from_sec():
    stamp = Stamp.from_sec(1.0)
    assert stamp.sec == 1.0
    assert stamp.nsec == 0

    stamp = Stamp.from_sec(1.5)
    assert stamp.sec == 1.0
    assert stamp.nsec == 500000000

    stamp = Stamp.from_sec(0.5)
    assert stamp.sec == 0.0
    assert stamp.nsec == 500000000


def test_from_nsec():
    stamp = Stamp.from_nsec(1)
    assert stamp.sec == 0.0
    assert stamp.nsec == 1

    stamp = Stamp.from_nsec(1000000000)
    assert stamp.sec == 1.0
    assert stamp.nsec == 0

    stamp = Stamp.from_nsec(500000000)
    assert stamp.sec == 0.0
    assert stamp.nsec == 500000000

    stamp = Stamp.from_nsec(1500000000)
    assert stamp.sec == 1.0
    assert stamp.nsec == 500000000


def test_comparisons():
    assert Stamp.from_sec(1.0) == Stamp.from_sec(1.0)
    assert Stamp.from_sec(1.0) != Stamp.from_sec(2.0)
    assert Stamp.from_sec(1.0) < Stamp.from_sec(2.0)
    assert Stamp.from_sec(2.0) > Stamp.from_sec(1.0)


# TODO: Test subtractions when Duration class is added
