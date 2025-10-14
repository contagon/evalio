from typing import TYPE_CHECKING, Annotated, TypeAlias, Literal

from cyclopts import Parameter
from evalio import datasets as ds, pipelines as pl

# ------------------------- Type aliases ------------------------- #
if TYPE_CHECKING:
    Sequences = str
    Pipelines = str
else:
    # TODO: Add star option to these literals
    # It keeps escaping funny!
    datasets = list(ds.all_sequences().keys())
    # datasets = []
    # datasets.extend([d + "/" + "\x5c" + "*" for d in ds.all_datasets().keys()])
    # print(datasets)
    Sequences = Literal[tuple(datasets)]
    Pipelines = Literal[tuple(pl.all_pipelines().keys())]

# TODO: Converter / Validator / no show
# TODO: Open a bug report, show_choices=False removes choices from completions
# TODO: Doesn't allow completing multiples of the same choice
DatasetArg: TypeAlias = Annotated[list[Sequences], Parameter()]
PipelineArg: TypeAlias = Annotated[list[Pipelines], Parameter()]
