from typing import TYPE_CHECKING, Literal

from evalio import datasets as ds, pipelines as pl

# ------------------------- Type aliases ------------------------- #
if TYPE_CHECKING:
    DataSequence = str
    Pipeline = str
else:
    # TODO: Add star option to these literals
    # It keeps escaping funny!
    datasets = list(ds.all_sequences().keys())
    # datasets = []
    # datasets.extend([d + "/" + "\x5c" + "*" for d in ds.all_datasets().keys()])
    # print(datasets)
    DataSequence = Literal[tuple(datasets)]
    Pipeline = Literal[tuple(pl.all_pipelines().keys())]

# TODO: Converter / Validator / no show
