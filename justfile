pybuild:
    touch pyproject.toml
    uv --verbose sync
    uv run pybind11-stubgen --numpy-array-wrap-with-annotated evalio._cpp -o python --ignore-all-errors
