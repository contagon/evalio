# Snapshot
- 2026-08-27 [USER] Configure Zensical to continue publishing versioned documentation through the existing `mike` `gh-pages` workflow.
- 2026-08-27 [CODE] `mike` is installed from Squidfunk's Zensical-compatible fork; CI deploys `latest` from `main` and release versions with the `stable` alias.
- 2026-08-27 [CODE] `project.extra.version.provider = "mike"` enables Zensical's version selector.
- 2026-08-27 [CODE] `project.site_url` uses the canonical HTTPS GitHub Pages URL with a trailing slash, so mike generates correct version URLs.

# Done (recent)
- 2026-08-27 [CODE] Corrected Zensical's canonical `site_url` for GitHub Pages and retained the existing mike aliases and deployment workflow.
- 2026-08-27 [CODE] Completed the Zensical migration: navigation, generated reference pages, Markdown extensions, and the mike version selector are configured.
- 2026-08-27 [CODE] Added unique Lucide front-matter icons to authored and generated documentation pages.
- 2026-08-27 [CODE] Completed palette-aware compact and wordmark SVG logo generation and site integration.

# Working set
- `zensical.toml`
- `.github/workflows/reusable_docs.yml`
- `pyproject.toml`

# Receipts
- 2026-08-27 [TOOL] `uv run python scripts/gen_docs.py && uv run zensical build` completed with no issues; generated canonical links use `https://contagon.github.io/evalio/`.
- 2026-08-27 [TOOL] `uv run mike list` reports `latest`, `0.6.0 [stable]`, and prior published versions on `gh-pages`; the installed mike identifies itself as Zensical-compatible.
- 2026-08-27 [TOOL] Zensical builds completed with mkdocstrings, math, navigation, palette-aware logo assets, and all generated documentation pages.
