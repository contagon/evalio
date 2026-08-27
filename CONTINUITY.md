# Snapshot
- 2026-08-26 [USER] Recreate `logo.png` as a reproducible TikZ logo, generating PDF and SVG; exact raster aspect ratio is not required.
- 2026-08-26 [USER] Remove all visible H1 headings from the docs landing page.

# Done (recent)
- 2026-08-26 [CODE] Superseded the unavailable Material helper class with native `<h1 hidden>`, removing the visible `evalio` heading.
- 2026-08-26 [CODE] Suppressed Material's injected “About” heading with an accessible, visually hidden landing-page H1.
- 2026-08-26 [CODE] Moved the README tagline into the centered logo block while preserving its emphasized acronym.
- 2026-08-26 [CODE] Centered both docs logo image display modes and the immediately adjacent tagline paragraph.
- 2026-08-26 [CODE] Centered the README logo with GitHub-compatible alignment and the docs logo with `margin-inline: auto`.
- 2026-08-26 [CODE] Added adaptive logos to `README.md` via `<picture>` and to the docs landing page via Material palette-aware CSS.
- 2026-08-26 [CODE] Replaced the same-color IMU `\shade` with `\fill[ink]`, preventing dvisvgm from dropping the center rectangle.
- 2026-08-26 [CODE] Added a `\darklogo` compile flag; `just logo` now builds dark-ink `logo.*` and light-ink `logo-dark.*` assets.
- 2026-08-26 [CODE] Added `just logo`, generating `docs/assets/logo.pdf` and `docs/assets/logo.svg` and removing LaTeX auxiliary files.
- 2026-08-26 [CODE] Replaced white marker/body strokes with true geometric gaps: calculated ring arcs, clipped IMU pins, and an inset chip body.
- 2026-08-26 [CODE] Added a 1pt white outline to the central IMU body and regenerated both vector outputs.
- 2026-08-26 [CODE] Superseded the segmented-ring experiment: restored the ring and IMU pins, with white outlines on orange returns.
- 2026-08-26 [CODE] Replaced shaded lidar returns with flat orange discs and redrew the IMU using slimmer pins and explicit axis geometry.
- 2026-08-26 [CODE] Added `logo.tex`, a tightly cropped standalone TikZ recreation of the lidar ring, returns, IMU glyph, and wordmark.
- 2026-08-26 [TOOL] LuaLaTeX generated `logo.pdf`; dvisvgm generated portable `logo.svg` with an embedded WOFF2 wordmark font; rendered preview verified.
- 2026-08-25 [CODE] Migrated docs to Zensical: dev deps swapped for `zensical` + mike fork; `mkdocs-gen-files` replaced by `scripts/gen_docs.py`; CI builds via `zensical build`. Site verified locally (tables, KaTeX, admonitions, mkdocstrings, version selector).
- 2026-08-25 [CODE] Zensical quirk: pipe table needs a blank line before it; generator writes `intro\n\n` to satisfy it.
- 2026-08-25 [CODE] Generator lives outside `docs_dir` (`scripts/`) so Zensical doesn't publish it into `site/`.

# Working set
- `pyproject.toml`
- `mkdocs.yml`
- `scripts/gen_docs.py`
- `.github/workflows/reusable_docs.yml`
- `justfile`

# Receipts
- 2026-08-26 [TOOL] `uv run mkdocs build --strict` passed; homepage contains `<h1 hidden>evalio</h1>` and no injected “About” H1.
- 2026-08-26 [TOOL] `uv run mkdocs build --strict` passed; built homepage has only `<h1 class="md-visually-hidden">evalio</h1>` and no “About” H1.
- 2026-08-26 [TOOL] `uv run mkdocs build --strict` passed; generated CSS includes image auto margins and centered adjacent-paragraph styling.
- 2026-08-26 [TOOL] `uv run mkdocs build --strict` passed after centering; generated `tweaks.css` contains the auto-margin rule.
- 2026-08-26 [TOOL] `uv run mkdocs build --strict` passed; built index references both SVGs, palette CSS is published, and both assets are present in `site/assets/`.
- 2026-08-26 [TOOL] Rebuilt both variants; SVGs now contain the rounded IMU body in `#181e24`/`#f5f7fa` and render correctly on light/dark backgrounds.
- 2026-08-26 [TOOL] Both PDF/SVG variants validated and rendered correctly on white and `#10151B` backgrounds; SVG ink colors are `#181e24` and `#f5f7fa`.
- 2026-08-26 [TOOL] `just logo` completed; both docs assets validate, SVG contains no white paint, and `docs/assets/` contains no auxiliary files.
- 2026-08-26 [TOOL] Rebuilt PDF/SVG, confirmed `logo.svg` contains no white paint, and previewed the transparent gaps on white and purple backgrounds.
- 2026-08-26 [TOOL] Previewed the outlined IMU body; LuaLaTeX and dvisvgm completed successfully.
- 2026-08-26 [TOOL] Rebuilt and previewed restored geometry with 2pt white outlines on orange lidar returns; PDF/SVG conversion succeeded.
- 2026-08-26 [TOOL] Rebuilt and previewed PDF/SVG with transparent marker spacing; no background-colored halo is baked into either asset.
- 2026-08-26 [TOOL] Rebuilt PDF/SVG after the glyph refinement; raster preview confirms clean flat markers and non-overlapping coordinate axes.
- 2026-08-26 [TOOL] `logo.pdf` is one 497.975x136.885 pt vector page; `logo.svg` converted successfully with dvisvgm 3.2.1.
- 2026-08-25 [TOOL] Zensical documentation confirms `mkdocs.yml`, KaTeX, mkdocstrings, and versioned docs compatibility; `mkdocs-gen-files` issue #8 is open.
