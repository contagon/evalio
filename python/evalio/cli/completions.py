import itertools
from .parser import DatasetBuilder, PipelineBuilder
import typer
from rapidfuzz.process import extractOne

valid_datasets = list(
    itertools.chain.from_iterable(
        [seq.full_name for seq in d.sequences()] + [f"{d.dataset_name()}/*"]
        for d in DatasetBuilder._all_datasets().values()
    )
)


def complete_dataset(incomplete: str):
    for name in valid_datasets:
        if name.startswith(incomplete):
            yield name


def validate_datasets(datasets: list[str]):
    for dataset in datasets:
        if dataset not in valid_datasets:
            closest, score, _idx = extractOne(dataset, valid_datasets)
            print(score)
            if score < 80:
                msg = dataset
            else:
                msg = f"{dataset}\n\nDid you mean {closest}?"
            raise typer.BadParameter(msg, param_hint="dataset")

    return datasets


valid_pipelines = list(PipelineBuilder._all_pipelines().keys())


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
                msg = f"{pipeline}\n\nDid you mean {closest}?"
            raise typer.BadParameter(msg, param_hint="pipeline")
