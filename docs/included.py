from typing import Optional, cast

import mkdocs_gen_files
from evalio.cli.ls import ls
from rich.table import Table
import re


def clean_cell(cell: str) -> str:
    """Clean a cell by removing unwanted characters."""
    # Convert to links
    if cell.startswith("[link"):
        match = re.match(r"\[link=(.*?)\](.*?)\[/link\]", cell)
        if match:
            url, text = match.groups()
            cell = f"[{text}]({url})"
            return cell

    # Remove rich text formatting
    cell = cell.replace("[bright_black]", "").replace("[/bright_black]", "")
    # Remove line breaks
    cell = cell.replace("\n", "<br>")
    # Non line breaking space
    cell = cell.replace(" ", "&nbsp;")
    # Non line breaking hyphen
    cell = cell.replace("-", "&#8209;")

    return cell.strip()


def center_code(kind: str) -> str:
    """Convert a justification code to a markdown table alignment code."""
    match kind:
        case "left":
            return ":--"
        case "center":
            return ":--:"
        case "right":
            return "--:"
        case _:
            return "---"


def rich_table_to_markdown(
    table: Table, skip_columns: Optional[list[str]] = None
) -> str:
    """Convert a rich Table to a markdown table string."""
    if skip_columns is None:
        skip_columns = []

    # Remove unwanted columns
    good_columns = [c for c in table.columns if c.header not in skip_columns]

    # Gather all of the data
    headers = [cast(str, column.header) for column in good_columns]
    bars = [center_code(column.justify) for column in good_columns]
    columns = [
        [clean_cell(cast(str, c)) for c in column._cells] for column in good_columns
    ]
    rows = zip(*columns)

    # Create markdown table header
    markdown = "| " + " | ".join(headers) + " |\n"
    markdown += "| " + " | ".join(bars) + " |\n"

    # Add rows to markdown table
    for row in rows:
        markdown += "| " + " | ".join(row) + " |\n"

    return markdown


DATASETS = """---
hide:
  - toc
---
evalio comes with a variety of datasets that can be used for easy loading. Below is a table of all datasets that are included, which mirrors the output of `evalio ls datasets`.
"""

with mkdocs_gen_files.open("included/datasets.md", "w") as f:
    f.write(DATASETS)
    f.write("\n")

    table = ls("datasets", show=False)
    if table is not None:
        f.write(rich_table_to_markdown(table, skip_columns=["DL", "Size"]))


PIPELINES = """---
hide:
  - toc
---
evalio comes with a variety of pipelines that can be used for evaluation. Below is a table of all pipelines that are included and their parameters, which mirrors the output of `evalio ls pipelines`.
"""

with mkdocs_gen_files.open("included/pipelines.md", "w") as f:
    f.write(PIPELINES)
    f.write("\n")

    table = ls("pipelines", show=False)
    if table is not None:
        f.write(rich_table_to_markdown(table))
