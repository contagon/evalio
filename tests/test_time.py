from evalio.types import Duration, Stamp


def test_stamp_from_sec():
    stamp = Stamp.from_sec(1.0)
    assert stamp.sec == 1.0
    assert stamp.nsec == 0

    stamp = Stamp.from_sec(1.5)
    assert stamp.sec == 1.0
    assert stamp.nsec == 500000000

    stamp = Stamp.from_sec(0.5)
    assert stamp.sec == 0.0
    assert stamp.nsec == 500000000


def test_stamp_from_nsec():
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


def test_stamp_comparisons():
    assert Stamp.from_sec(1.0) == Stamp.from_sec(1.0)
    assert Stamp.from_sec(1.0) != Stamp.from_sec(2.0)
    assert Stamp.from_sec(1.0) < Stamp.from_sec(2.0)
    assert Stamp.from_sec(2.0) > Stamp.from_sec(1.0)


def test_duration_from_sec():
    stamp = Duration.from_sec(1.0)
    assert stamp.nsec == 1e9

    stamp = Duration.from_sec(1.5)
    assert stamp.nsec == 1500000000

    stamp = Duration.from_sec(0.5)
    assert stamp.nsec == 500000000


def test_duration_from_nsec():
    stamp = Duration.from_nsec(1)
    assert stamp.nsec == 1

    stamp = Duration.from_nsec(1000000000)
    assert stamp.nsec == 1000000000

    stamp = Duration.from_nsec(500000000)
    assert stamp.nsec == 500000000

    stamp = Duration.from_nsec(1500000000)
    assert stamp.nsec == 1500000000


def test_duration_comparisons():
    assert Duration.from_sec(1.0) == Duration.from_sec(1.0)
    assert Duration.from_sec(1.0) != Duration.from_sec(2.0)
    assert Duration.from_sec(1.0) < Duration.from_sec(2.0)
    assert Duration.from_sec(2.0) > Duration.from_sec(1.0)


def test_operators():
    stamp = Stamp.from_sec(1.0)
    duration = Duration.from_sec(0.5)

    assert stamp + duration == Stamp.from_sec(1.5)
    assert stamp - duration == Stamp.from_sec(0.5)

    stamp2 = Stamp.from_sec(0.25)
    assert stamp - stamp2 == Duration.from_sec(0.75)
