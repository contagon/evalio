pybuild:
    touch pyproject.toml
    uv --verbose sync --all-extras
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

stubs:
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

bump-minor:
    uv run bump-my-version minor
    git push --tags

bump-patch:
    uv run bump-my-version patch
    git push --tags