import mkdocs_gen_files
from evalio.cli.ls import ls, Kind
from rich.table import Table
from typing import Optional, cast


def clean_cell(cell: str) -> str:
    """Clean a cell by removing unwanted characters."""
    # Remove line breaks
    cell = cell.replace("\n", "<br>")
    # Convert to links
    if cell.startswith("http"):
        shorter = cell.split("//", maxsplit=1)[-1]
        cell = f"[{shorter}]({cell})"

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


with mkdocs_gen_files.open("included/datasets.md", "w") as f:
    table = ls(Kind.datasets, show=False)
    if table is not None:
        f.write(rich_table_to_markdown(table, skip_columns=["Down"]))


with mkdocs_gen_files.open("included/pipelines.md", "w") as f:
    table = ls(Kind.pipelines, show=False)
    if table is not None:
        f.write(rich_table_to_markdown(table))
