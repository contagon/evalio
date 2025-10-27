from typing import TYPE_CHECKING, Literal, Sequence

from cyclopts import Token
from evalio import datasets as ds, pipelines as pl

from rapidfuzz.process import extractOne

# ------------------------- Type aliases ------------------------- #
all_sequences = list(ds.all_sequences().keys())
all_sequences.extend([d + "/*" for d in ds.all_datasets().keys()])
all_pipelines = list(pl.all_pipelines().keys())

if TYPE_CHECKING:
    DataSeq = str
    Pipeline = str
else:
    DataSeq = Literal[tuple(all_sequences)]
    Pipeline = Literal[tuple(all_pipelines)]


# This is really a validator, but I want to shortcut the built-in Literal validation
def data_sequence_converter(type_: type, value: Sequence[Token]) -> list[str]:
    for v in value:
        if v.value not in ds.all_sequences():
            # closest, score, _idx
            out = extractOne(v.value, ds.all_sequences().keys())
            if out is None or out[1] < 80:
                msg = v.value
            else:
                msg = f"{v.value}\nA similar seq exists: {out[0]}"

            if v.keyword is None:
                msg = f"Invalid data sequence: {msg}"

            raise ValueError(msg)

    return [t.value for t in value]


def pipeline_converter(type_: type, value: Sequence[Token]) -> list[str]:
    for v in value:
        if v.value not in pl.all_pipelines():
            # closest, score, _idx
            out = extractOne(v.value, pl.all_pipelines().keys())
            if out is None or out[1] < 80:
                msg = v.value
            else:
                msg = f"{v.value}\nA similar pipeline exists:  {out[0]}"

            raise ValueError(f"Invalid pipeline: {msg}")

    return [t.value for t in value]
