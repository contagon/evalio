from .parser import DatasetBuilder, PipelineBuilder
from typing import Optional
import typer
from typing_extensions import Annotated
from enum import StrEnum, auto
from rapidfuzz.process import extract_iter

from rich.console import Console
from rich.table import Table
from rich import box

app = typer.Typer()


class Kind(StrEnum):
    datasets = auto()
    pipelines = auto()


@app.command(no_args_is_help=True)
def ls(
    kind: Annotated[
        Kind, typer.Argument(help="The kind of object to list", show_default=False)
    ],
    search: Annotated[
        Optional[str],
        typer.Option(
            "--search",
            "-s",
            help="Fuzzy search for a pipeline or dataset by name",
            show_default=False,
        ),
    ] = None,
    quiet: Annotated[
        bool,
        typer.Option(
            "--quiet",
            "-q",
            help="Output less verbose information",
        ),
    ] = False,
):
    """
    List dataset and pipeline information
    """
    if kind == Kind.datasets:
        # Search for datasets using rapidfuzz
        # TODO: Make it search through sequences as well?
        all_datasets = list(DatasetBuilder._all_datasets().values())
        if search is not None:
            to_include = extract_iter(
                search, [d.dataset_name() for d in all_datasets], score_cutoff=90
            )
            to_include = [all_datasets[idx] for _name, _score, idx in to_include]
        else:
            to_include = all_datasets

        # For future self: To add a new column, the following needs to be done:
        # 1. Add the column to all_info
        # 2. Gather the info for that column in the for loop
        # 3. Add the column to the table
        # That should be about it, making the rest should be automatic

        # Gather all info
        all_info = {"Name": [], "Sequences": [], "Down": [], "More Info": []}
        for d in to_include:
            all_info["Name"].append(d.dataset_name())
            all_info["More Info"].append(d.url())

            if not quiet:
                all_info["Sequences"].append("\n".join(d.sequences()))
                downloaded = [d(s).is_downloaded() for s in d.sequences()]
                downloaded = "\n".join(
                    ["[green]✔[/green]" if d else "[red]-[/red]" for d in downloaded]
                )
                all_info["Down"].append(downloaded)

        if len(all_info["Name"]) == 0:
            print("No datasets found")
            return

        # Fill out table
        table = Table(
            title="Datasets",
            show_lines=not quiet,
            highlight=True,
            box=box.ROUNDED,
        )
        col_opts = {"vertical": "middle"}

        table.add_column("Name", justify="center", **col_opts)  # type: ignore
        if not quiet:
            table.add_column("Sequences", justify="right", **col_opts)  # type: ignore
            table.add_column("Down", justify="center", **col_opts)  # type: ignore
        table.add_column("More Info", justify="center", **col_opts)  # type: ignore

        for i in range(len(all_info["Name"])):
            row_info = [all_info[c.header][i] for c in table.columns]  # type: ignore
            table.add_row(*row_info)

        Console().print(table)

    if kind == Kind.pipelines:
        # Search for pipelines using rapidfuzz
        # TODO: Make it search through parameters as well?
        all_pipelines = list(PipelineBuilder._all_pipelines().values())
        if search is not None:
            to_include = extract_iter(
                search, [d.name() for d in all_pipelines], score_cutoff=90
            )
            to_include = [all_pipelines[idx] for _name, _score, idx in to_include]
        else:
            to_include = all_pipelines

        # For future self: To add a new column, the following needs to be done:
        # 1. Add the column to all_info
        # 2. Gather the info for that column in the for loop
        # 3. Add the column to the table
        # That should be about it, making the rest should be automatic

        # Gather all info
        all_info = {"Name": [], "Params": [], "Default": [], "More Info": []}
        for p in to_include:
            all_info["Name"].append(p.name())
            all_info["More Info"].append(p.url())

            if not quiet:
                params = p.default_params()
                keys = "\n".join(params.keys())
                values = "\n".join([str(v) for v in params.values()])
                all_info["Params"].append(keys)
                all_info["Default"].append(values)

        if len(all_info["Name"]) == 0:
            print("No pipelines found")
            return

        # Fill out table
        table = Table(
            title="Pipelines",
            show_lines=not quiet,
            highlight=True,
            box=box.ROUNDED,
        )
        col_opts = {"vertical": "middle"}

        table.add_column("Name", justify="center", **col_opts)  # type: ignore
        if not quiet:
            table.add_column("Params", justify="right", **col_opts)  # type: ignore
            table.add_column("Default", justify="left", **col_opts)  # type: ignore
        table.add_column("More Info", justify="center", **col_opts)  # type: ignore

        for i in range(len(all_info["Name"])):
            row_info = [all_info[c.header][i] for c in table.columns]  # type: ignore
            table.add_row(*row_info)

        Console().print(table)
