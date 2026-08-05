# AGENTS.md

## Project Overview

### Commands

- **Sync deps:** `uv sync --dev --all-extras`
- **Rebuild C++ extension:** `just pybuild` (touches `pyproject.toml`, runs `uv sync`, copies the compiled `_cpp` from `site-packages` into `python/evalio/`)
- **Test:** `uv run pytest -v`
- **Typecheck:** `uv run basedpyright`
- **Lint:** `ruff check`

### Architecture

- Python package lives in `python/evalio/`; C++ nanobind extension source in `cpp/` (bindings in `cpp/bindings/`)
- `tests/` is the only pytest `testpaths`
- `python/typings/` holds vendored type stubs (`basedpyright` `stubPath`)
- The C++ portion builds lidar odometry pipelines as well, they are stored in `cpp/bindings/pipelines-src` and should not be edited directly without confirmation.

### Local Dependencies

No local sibling dependencies; all deps are installed from PyPI.

### Quirks

- After any C++ change (`cpp/**` or `CMakeLists.txt`), run `just pybuild` to rebuild and refresh the compiled `_cpp` in `python/evalio/`
- CI installs the package as a wheel (`UV_NO_EDITABLE=true`), not editable

## Accuracy, recency, and sourcing (REQUIRED)


### Editing files

- Make the smallest safe change that solves the issue.
- Preserve existing style and conventions.
- Prefer patch-style edits (small, reviewable diffs) over full-file rewrites.
- After making changes, run the project’s standard checks when feasible (format/lint, unit tests, build/typecheck).
- Never remove comments unless there are no longer valid
- NEVER utilize git stash or other git commands that would remove local changes
- Keep all code comments to two lines and under. Only comment for mathematical explanations or clarifying difficult code.

## Baseline workflow

- Start every task by determining:
  1. Goal + acceptance criteria.
  2. Constraints (time, safety, scope).
  3. What must be inspected (files, commands, tests, docs).
  4. Whether the request depends on **recency** (if yes, apply the "Accuracy, recency, and sourcing" rules).
  5. If requirements are ambiguous, ask targeted clarifying questions before making irreversible changes.


## Continuity Ledger (compaction-safe)

Maintain a single continuity file for this workspace: `CONTINUITY.md`.
`CONTINUITY.md` is the canonical briefing designed to survive compaction; do not rely on earlier chat/tool output unless it's reflected there.

### Operating rule
- At the start of each assistant turn: read `CONTINUITY.md` before acting.
- Update `CONTINUITY.md` only when there is a meaningful delta in: Goal/success criteria, Invariants/constraints, Decisions, State (Done/Now/Next), Open questions, Working set, or important tool outcomes.

### Keep it bounded (anti-bloat)
- Keep `CONTINUITY.md` short and high-signal:
  - `Snapshot`: ≤ 25 lines.
  - `Done (recent)`: ≤ 7 bullets.
  - `Working set`: ≤ 12 paths.
  - `Receipts`: keep last 10–20 entries.
- If sections exceed caps, compress older items into milestone bullets with pointers (commit/PR/log path/doc path). Do not paste raw logs.

### Anti-drift rules
- Facts only, no transcripts.
- Every entry must include:
  - a date or ISO timestamp (e.g., `2026-01-13` or `2026-01-13T09:42Z`)
  - a provenance tag: `[USER]`, `[CODE]`, `[TOOL]`, `[ASSUMPTION]`
- If unknown, write `UNCONFIRMED` (never guess). If something changes, supersede it explicitly (don't silently rewrite history).

### Decisions and incidents
- Record durable choices in `Decisions` as ADR-lite entries (e.g., `D001 ACTIVE: …`).
- For recurring weirdness, create a small, stable incident capsule (Symptoms / Evidence pointers / Mitigation / Status).

### Plan tool vs ledger
- Use `update_plan` for short-term execution scaffolding (3–7 steps).
- Use `CONTINUITY.md` for long-running continuity ("what/why/current state"), not micro task lists.
- Keep them consistent at the intent/progress level.

### In replies
- Start with a brief "Ledger Snapshot" (Goal + Now + Next + Open Questions).
- Print the full ledger only when it materially changed or the user requests it.