from tabulate import tabulate

from .parser import DatasetBuilder, PipelineBuilder
from typing import Optional
import typer
from typing_extensions import Annotated
from enum import StrEnum, auto
from rapidfuzz.process import extract_iter

app = typer.Typer()


class Kind(StrEnum):
    datasets = auto()
    pipelines = auto()


@app.command(no_args_is_help=True)
def ls(
    kind: Annotated[Kind, typer.Argument(help="The kind of object to list")],
    search: Annotated[
        Optional[str],
        typer.Option(
            "--search",
            "-s",
            help="Fuzzy search for a dataset by name",
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
        data = [["Name", "Sequences", "Down", "Link"]]

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

        # Fill out table
        for d in to_include:
            seq = "\n".join(d.sequences())
            downloaded = [d(s).is_downloaded() for s in d.sequences()]
            downloaded = "\n".join(["✔" if d else "-" for d in downloaded])
            data.append([d.dataset_name(), seq, downloaded, d.url()])

        if len(data) == 1:
            print("No datasets found")
            return

        align = ("center", "right", "center", "center")

        # delete unneeded columns if quiet
        if quiet:
            data = [[d[0], d[3]] for d in data]
            align = ("center", "center")

        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=align,
            )
        )

    if kind == Kind.pipelines:
        data = [["Name", "Params", "Default", "Link"]]

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

        # Fill out table
        for p in to_include:
            params = p.default_params()
            keys = "\n".join(params.keys())
            values = "\n".join([str(v) for v in params.values()])
            data.append([p.name(), keys, values, p.url()])

        if len(data) == 1:
            print("No pipelines found")
            return

        align = ("center", "right", "left", "center")

        # delete unneeded columns if quiet
        if quiet:
            data = [[d[0], d[3]] for d in data]
            align = ("center", "center")

        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=align,
            )
        )
