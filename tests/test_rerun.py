import pytest
from evalio import rerun
from evalio._cpp.types import VisOption  # type: ignore


def test_rerun_is_optional(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(rerun, "rr", None)
    vis = rerun.RerunVis({VisOption.SCAN}, [])
    assert vis.args is None

    with pytest.raises(ImportError, match=r"evalio\[vis\]"):
        rerun.convert([])