import itertools
from typing import TypeAlias, Optional
from .parser import DatasetBuilder, PipelineBuilder
import typer
from rapidfuzz.process import extractOne
import importlib
from typing_extensions import Annotated


# ------------------------- Completions ------------------------- #
def valid_sequences(custom_modules: Optional[list[str]] = None):
    return list(
        itertools.chain.from_iterable(
            [seq.full_name for seq in d.sequences()] + [f"{d.dataset_name()}/*"]
            for d in DatasetBuilder._all_datasets(custom_modules).values()
        )
    )


def complete_dataset(incomplete: str, ctx: typer.Context):
    # TODO: Check for * to remove autocompletion for all of that dataset
    already_listed = ctx.params.get("datasets") or []
    custom_modules = ctx.params.get("modules", None)

    for name in valid_sequences(custom_modules):
        if name not in already_listed and name.startswith(incomplete):
            yield name


def validate_datasets(datasets: list[str], ctx: typer.Context):
    custom_modules = ctx.params.get("modules", None)
    all_seq = valid_sequences(custom_modules)

    for dataset in datasets:
        if dataset not in all_seq:
            closest, score, _idx = extractOne(dataset, all_seq)
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


# TODO: Upgrade pipeline autocompletion with custom modules like dataset
def complete_pipeline(incomplete: str):
    for name in valid_pipelines:
        if name.startswith(incomplete):
            yield name


def validate_pipelines(pipelines: list[str]):
    for pipeline in pipelines:
        if pipeline not in valid_pipelines:
            closest, score, _idx = extractOne(pipeline, valid_pipelines)
            print(score)
            if score < 80:
                msg = pipeline
            else:
                # TODO: color would be nice here, but breaks rich panel spacing
                # name = typer.style(closest, fg=typer.colors.RED)
                msg = f"{pipeline}\n A similar pipeline exists: {closest}"
            raise typer.BadParameter(msg, param_hint="pipeline")

    return pipelines


def validate_modules(modules: Optional[list[str]] = None):
    if modules is None:
        return []

    for module in modules:
        try:
            importlib.import_module(module)
        except ImportError:
            raise typer.BadParameter(
                f"Failed to import '{module}'", param_hint="module"
            )

    return modules


# ------------------------- Type aliases ------------------------- #

DatasetArg: TypeAlias = Annotated[
    list[str],
    typer.Argument(
        help="The dataset(s) to download",
        autocompletion=complete_dataset,
        callback=validate_datasets,
    ),
]

ModuleArg: TypeAlias = Annotated[
    Optional[list[str]],
    typer.Option(
        "--module",
        "-m",
        help="Additional python modules to search for datasets or pipelines",
        callback=validate_modules,
    ),
]
