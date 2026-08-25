pybuild:
    touch pyproject.toml
    uv --verbose sync --all-extras
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

stubs:
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

docs:
    uv run python scripts/gen_docs.py
    uv run zensical build

bump-minor:
    uv run bump-my-version bump minor
    git push --tags
    git push

bump-patch:
    uv run bump-my-version bump patch
    git push --tags
    git push
