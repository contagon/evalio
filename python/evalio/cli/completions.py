from typing import TypeAlias

from .parser import PipelineBuilder, DatasetBuilder
import typer
from rapidfuzz.process import extractOne
from typing import Annotated, Optional
import itertools

from rich.console import Console


err_console = Console(stderr=True)

all_sequences_names = list(
    itertools.chain.from_iterable(
        [seq.full_name for seq in d.sequences()] + [f"{d.dataset_name()}/*"]
        for d in DatasetBuilder._all_datasets().values()
    )
)


# ------------------------- Completions ------------------------- #
def complete_dataset(incomplete: str, ctx: typer.Context):
    # TODO: Check for * to remove autocompletion for all of that dataset
    already_listed = ctx.params.get("datasets") or []

    for name in all_sequences_names:
        if name not in already_listed and name.startswith(incomplete):
            yield name


def validate_datasets(datasets: list[str]):
    if not datasets:
        return []

    for dataset in datasets:
        if dataset not in all_sequences_names:
            closest, score, _idx = extractOne(dataset, all_sequences_names)
            if score < 80:
                msg = dataset
            else:
                # TODO: color would be nice here, but breaks rich panel spacing
                # name = typer.style(closest, fg=typer.colors.RED)
                msg = f"{dataset}\n A similar dataset exists: {closest}"
            raise typer.BadParameter(msg, param_hint="dataset")

    if len(set(datasets)) != len(datasets):
        raise typer.BadParameter("Duplicate datasets listed", param_hint="dataset")

    return datasets


valid_pipelines = list(PipelineBuilder._all_pipelines().keys())


def complete_pipeline(incomplete: str, ctx: typer.Context):
    already_listed = ctx.params.get("pipelines") or []

    for name in valid_pipelines:
        if name not in already_listed and name.startswith(incomplete):
            yield name


def validate_pipelines(pipelines: list[str]):
    if not pipelines:
        return []

    for pipeline in pipelines:
        if pipeline not in valid_pipelines:
            closest, score, _idx = extractOne(pipeline, valid_pipelines)
            if score < 80:
                msg = pipeline
            else:
                # TODO: color would be nice here, but breaks rich panel spacing
                # name = typer.style(closest, fg=typer.colors.RED)
                msg = f"{pipeline}\n A similar pipeline exists: {closest}"
            raise typer.BadParameter(msg, param_hint="pipeline")

    return pipelines


# ------------------------- Type aliases ------------------------- #

DatasetArg: TypeAlias = Annotated[
    list[str],
    typer.Argument(
        help="The dataset(s) to use",
        autocompletion=complete_dataset,
        callback=validate_datasets,
        show_default=False,
    ),
]

DatasetOpt: TypeAlias = Annotated[
    Optional[list[str]],
    typer.Option(
        "--datasets",
        "-d",
        help="The dataset(s) to use",
        autocompletion=complete_dataset,
        callback=validate_datasets,
        rich_help_panel="Manual options",
        show_default=False,
    ),
]

PipelineArg: TypeAlias = Annotated[
    list[str],
    typer.Argument(
        help="The pipeline(s) to use",
        autocompletion=complete_pipeline,
        callback=validate_pipelines,
        show_default=False,
    ),
]

PipelineOpt: TypeAlias = Annotated[
    Optional[list[str]],
    typer.Option(
        "--pipelines",
        "-p",
        help="The pipeline(s) to use",
        autocompletion=complete_pipeline,
        callback=validate_pipelines,
        rich_help_panel="Manual options",
        show_default=False,
    ),
]
