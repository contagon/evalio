from typer.cli import app
from contextlib import redirect_stdout
import mkdocs_gen_files

with mkdocs_gen_files.open("ref/cli.md", "w") as f:
    with redirect_stdout(f):
        app(["evalio.cli", "utils", "docs", "--name", "evalio"])
