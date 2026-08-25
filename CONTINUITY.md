# CONTINUITY.md

## Snapshot
- Goal: Make rerun visualization optional at runtime (no hard import), with full frame support (parent/child) in rerun logs.
- Branch: `rerun-updates`, base `4ca115e`.
- Now: Working tree has 1 modified file (`python/evalio/rerun.py`) + new test (`tests/test_rerun.py`), all uncommitted.
- Next: Verify with `uv run pytest -v tests/test_rerun.py`; typecheck; run project test suite.

## Done (recent)
- Rerun import made lazy via `_load_rerun()`; `rr`/`rrb` optional at runtime.
- Moved `OverrideType`/`RerunArgs` out of the import guard (no rerun type deps).
- `RerunVis` degrades gracefully (prints warning, disables) when rerun absent; removed ImportError fallback class.
- Added `parent_frame`/`child_frame` support to `convert()` and threaded frames through all `rec.log` calls.
- `convert()` raises clear `ImportError` suggesting `pip install evalio[vis]` when rerun missing.
- Added `tests/test_rerun.py::test_rerun_is_optional`.

## Decisions
- D001 ACTIVE: Rerun is now optional-first; no hard dependency at import. Behavior folded into single `RerunVis` class instead of a duplicate fallback.

## Working set
- `python/evalio/rerun.py`
- `tests/test_rerun.py`
- `.kiro/` (untracked scratch, ignore)

## Receipts
- 2026-01-13 [TOOL] Created CONTINUITY.md to track `rerun-updates` work.
- 2026-01-13 [TOOL] `git status`: 1 modified (rerun.py), 2 untracked (test_rerun.py, .kiro/); base commit 4ca115e.