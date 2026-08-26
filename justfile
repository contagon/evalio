pybuild:
    touch pyproject.toml
    uv --verbose sync --all-extras
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

stubs:
    cp -r .venv/lib/python3.12/site-packages/evalio/_cpp python/evalio

logo:
    mkdir -p docs/assets
    lualatex -interaction=nonstopmode -halt-on-error -output-directory=docs/assets docs/assets/logo.tex
    lualatex -interaction=nonstopmode -halt-on-error -jobname=logo-dark -output-directory=docs/assets '\def\darklogo{1}\input{docs/assets/logo.tex}'
    dvisvgm --pdf --font-format=woff2 --output=docs/assets/logo.svg docs/assets/logo.pdf
    dvisvgm --pdf --font-format=woff2 --output=docs/assets/logo-dark.svg docs/assets/logo-dark.pdf
    rm docs/assets/logo.aux docs/assets/logo.log docs/assets/logo-dark.aux docs/assets/logo-dark.log

bump-minor:
    uv run bump-my-version bump minor
    git push --tags
    git push

bump-patch:
    uv run bump-my-version bump patch
    git push --tags
    git push