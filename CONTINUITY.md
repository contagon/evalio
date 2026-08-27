# Snapshot
- 2026-08-27 [USER] Restore Zensical migration settings and fix KaTeX rendering plus generated Python API documentation.
- 2026-08-27 [CODE] `zensical.toml` now represents those MkDocs settings; `main.md` replaces the obsolete `index.md` landing path.

# Done (recent)
- 2026-08-27 [CODE] Restored `[project.extra.version] provider = "mike"`, asset lists, `watch`, explicit navigation labels, and the Included section.
- 2026-08-27 [CODE] Moved the mike TOML table after project-level settings, restoring navigation and extra assets; configured mkdocstrings' Python handler; separated the Odometry display equation into a Markdown block.
- 2026-08-27 [CODE] Removed `content.code.select`, disabling the code-block line-selection modal.
- 2026-08-27 [CODE] Added one-line descriptions for every active Markdown extension setting in `zensical.toml`.
- 2026-08-27 [CODE] Removed unused Markdown extensions and the unused Mermaid fence, retaining inline highlighting, code highlighting, math, admonitions, nested fences, and tabs.
- 2026-08-26 [CODE] Docs migration, adaptive logo assets, and hidden landing-page H1 completed.

# Working set
- `zensical.toml`
- `mkdocs.yml`
- `docs/`

# Receipts
- 2026-08-27 [TOOL] `uv run zensical build` completed; generated configuration contains `provider: "mike"`, restored nav, CSS, and JavaScript lists.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues; Odometry emits `.arithmatex` markup and datasets/pipelines emit mkdocstrings documentation objects.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after disabling line selection.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after documenting Markdown extensions.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after trimming Markdown extensions.
- 2026-08-27 [TOOL] Build reported six pre-existing broken page links, including absent `docs/included/{datasets,pipelines}.md` targets.
- 2026-08-26 [TOOL] Logo PDF/SVG variants rendered correctly on light and dark backgrounds.
