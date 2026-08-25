# Snapshot
- 2026-08-25 [USER] Migrate documentation from MkDocs Material to Zensical and retain generated documentation in CI.
- 2026-08-25 [CODE] Zensical supports the current configuration except `mkdocs-gen-files`; use `docs/generate.py` as a prebuild replacement.

# Done (recent)
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
- 2026-08-25 [TOOL] Zensical documentation confirms `mkdocs.yml`, KaTeX, mkdocstrings, and versioned docs compatibility; `mkdocs-gen-files` issue #8 is open.
