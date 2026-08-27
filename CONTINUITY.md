# Snapshot
- 2026-08-27 [USER] Add Lucide icon front matter to all documentation pages, matching `quickstart.md`; generated pages must receive headers from `scripts/gen_docs.py`.
- 2026-08-27 [CODE] Added icons to every authored page and configured generated Included and CLI pages to retain theirs after regeneration.
- 2026-08-27 [CODE] Every page now has a distinct Lucide icon; `database` remains only for the custom-dataset guide.
- 2026-08-27 [ASSUMPTION] Existing user changes to logo assets, styles, configuration, and `quickstart.md` are unrelated and must remain untouched.

# Done (recent)
- 2026-08-27 [CODE] Replaced duplicated database, chart, and workflow icons with distinct page-specific Lucide icons.
- 2026-08-27 [CODE] Added concise Lucide icon front matter to authored pages and `scripts/gen_docs.py` generated output.
- 2026-08-27 [CODE] Restored `[project.extra.version] provider = "mike"`, asset lists, `watch`, explicit navigation labels, and the Included section.
- 2026-08-27 [CODE] Moved the mike TOML table after project-level settings, restoring navigation and extra assets; configured mkdocstrings' Python handler; separated the Odometry display equation into a Markdown block.
- 2026-08-27 [CODE] Removed `content.code.select`, disabling the code-block line-selection modal.
- 2026-08-27 [CODE] Added one-line descriptions for every active Markdown extension setting in `zensical.toml`.
- 2026-08-27 [CODE] Removed unused Markdown extensions and the unused Mermaid fence, retaining inline highlighting, code highlighting, math, admonitions, nested fences, and tabs.
- 2026-08-27 [CODE] Set `project.extra.homepage` to `/evalio/about/`, directing the header and sidebar logo to the About page.
- 2026-08-27 [CODE] `just logo` produces icon-only `logo*` and wordmark `logo-wordmark*` light/dark SVG assets, deleting all temporary PDFs after conversion.
- 2026-08-27 [CODE] Configured `assets/logo.svg` as the Zensical favicon and header/sidebar logo.
- 2026-08-27 [CODE] Added palette-aware compact-logo CSS and favicon synchronization, selecting `logo-dark.svg` for the slate palette.
- 2026-08-26 [CODE] Docs migration, adaptive logo assets, and hidden landing-page H1 completed.
- 2026-08-27 [CODE] Added `logo-small.tex` and rendered light/dark SVG previews without changing site configuration.
- 2026-08-27 [CODE] `just logo` now regenerates the compact SVG pair; the header, favicon, and palette switcher use `logo-small*`.
- 2026-08-27 [CODE] Refined the compact mark to a continuous lidar ring behind the returns and enlarged the central IMU chip and pins.
- 2026-08-27 [CODE] Shortened the compact mark's IMU pins and added a single hollow highlight circle at its center.

# Working set
- `zensical.toml`
- `docs/`
- `scripts/gen_docs.py`

# Receipts
- 2026-08-27 [TOOL] `uv run zensical build` completed; generated configuration contains `provider: "mike"`, restored nav, CSS, and JavaScript lists.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues; Odometry emits `.arithmatex` markup and datasets/pipelines emit mkdocstrings documentation objects.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after disabling line selection.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after documenting Markdown extensions.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after trimming Markdown extensions.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues; generated header and sidebar logo links target `/evalio/about/`.
- 2026-08-27 [TOOL] `just logo` and `uv run zensical build` completed; generated About page references the renamed wordmark SVG variants.
- 2026-08-27 [TOOL] `just logo` completed and `docs/assets/**/*.pdf` has no matches.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues; generated About page references `assets/logo.svg` for the favicon and both theme logos.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues; the built page publishes the palette switcher and dark-logo CSS.
- 2026-08-27 [TOOL] Build reported six pre-existing broken page links, including absent `docs/included/{datasets,pipelines}.md` targets.
- 2026-08-26 [TOOL] Logo PDF/SVG variants rendered correctly on light and dark backgrounds.
- 2026-08-27 [TOOL] LuaLaTeX and dvisvgm rendered `logo-small.svg` and `logo-small-dark.svg`; both are square 56.028pt assets.
- 2026-08-27 [TOOL] `just logo && uv run zensical build` completed; compact variants are square 103.585pt SVGs and no PDFs remain.
- 2026-08-27 [TOOL] `just logo` completed; corrected compact variants are square 109.275pt SVGs and no PDFs remain.
- 2026-08-27 [TOOL] `just logo` completed; compact SVGs are square 100.203pt and no PDFs remain.
- 2026-08-27 [TOOL] `scripts/gen_docs.py` generates Included datasets/pipelines and CLI reference Markdown; CI invokes it before `uv run zensical build`.
- 2026-08-27 [TOOL] `uv run python scripts/gen_docs.py && uv run zensical build && ruff check scripts/gen_docs.py` completed successfully.
- 2026-08-27 [TOOL] Regeneration, Zensical build, and lint passed; `rg` confirmed no duplicate Markdown icon values.
