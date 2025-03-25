pybuild:
    touch pyproject.toml
    uv --verbose sync --all-extras
    uv run pybind11-stubgen --numpy-array-wrap-with-annotated evalio._cpp -o python --ignore-all-errors

stubs:
    uv run pybind11-stubgen --numpy-array-wrap-with-annotated evalio._cpp -o python --ignore-all-errors

bump-minor:
    uv run bump-my-version minor
    git push --tags

bump-patch:
    uv run bump-my-version patch
    git push --tags