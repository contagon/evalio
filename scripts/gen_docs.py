from collections.abc import Set as AbstractSet
from pathlib import Path
from typing import Literal, cast

from cyclopts.docs.markdown import generate_markdown_docs
from evalio.cli import app
from evalio.cli.ls import ls
from rich.table import Table

DOCS = Path(__file__).parent.parent / "docs"


def clean_cell(cell: str) -> str:
    """Clean a cell by removing unwanted characters."""
    cell = cell.replace("[bright_black]", "").replace("[/bright_black]", "")
    cell = cell.replace("\n", "<br>")
    cell = cell.replace(" ", "&nbsp;")
    cell = cell.replace("-", "&#8209;")
    if cell.startswith("http"):
        cell = f"[link]({cell})"
    return cell.strip()


def rich_table_to_markdown(
    table: Table, skip_columns: AbstractSet[str] = frozenset()
) -> str:
    """Convert a rich Table to Markdown."""
    columns = [column for column in table.columns if column.header not in skip_columns]
    alignment = {"left": ":--", "center": ":--:", "right": "--:"}
    markdown = (
        "| " + " | ".join(cast(str, column.header) for column in columns) + " |\n"
    )
    markdown += (
        "| "
        + " | ".join(alignment.get(column.justify, "---") for column in columns)
        + " |\n"
    )
    for row in zip(
        *([clean_cell(cast(str, cell)) for cell in column._cells] for column in columns)
    ):
        markdown += "| " + " | ".join(row) + " |\n"
    return markdown


def write_included(
    kind: Literal["datasets", "pipelines"],
    filename: str,
    intro: str,
    icon: str,
    skip_columns: AbstractSet[str] = frozenset(),
) -> None:
    table = ls(kind, show=False, show_hyperlinks=True)
    content = f"---\nicon: lucide/{icon}\nhide:\n  - toc\n---\n{intro}\n\n"
    if table is not None:
        content += rich_table_to_markdown(table, skip_columns)
    path = DOCS / filename
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


write_included(
    "datasets",
    "included/datasets.md",
    "evalio comes with a variety of datasets that can be used for easy loading. Below is a table of all datasets that are included, which mirrors the output of `evalio ls datasets`.",
    "hard-drive",
    {"DL", "Size"},
)
write_included(
    "pipelines",
    "included/pipelines.md",
    "evalio comes with a variety of pipelines that can be used for evaluation. Below is a table of all pipelines that are included and their parameters, which mirrors the output of `evalio ls pipelines`.",
    "blocks",
)
# zensical does not run mkdocs plugins, so instead of the `::: cyclopts` directive we
# call the same generator the cyclopts mkdocs plugin wraps.
cli_path = DOCS / "ref/cli.md"
cli_path.parent.mkdir(parents=True, exist_ok=True)
cli_path.write_text(
    "---\nicon: lucide/terminal\n---\n\n"
    + generate_markdown_docs(
        app,
        heading_level=1,
        generate_toc=False,
        code_block_title=True,
    )
)
