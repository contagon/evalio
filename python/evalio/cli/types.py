from typing import TYPE_CHECKING, Annotated, Any, Optional, TypeAlias, Literal

from cyclopts import Group, Parameter
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
DatasetArg: TypeAlias = Annotated[list[Sequences], Parameter()]
PipelineArg: TypeAlias = Annotated[list[Pipelines], Parameter()]


def Param(
    alias: Optional[str] = None,
    group: Optional[Group] = None,
    **kwargs: dict[str, Any],
) -> Parameter:
    """Helper to create a Parameter with custom defaults.

    Args:
        alias (Optional[str], optional): _description_. Defaults to None.
        group (Optional[Group], optional): _description_. Defaults to None.
        name (Optional[str], optional): _description_. Defaults to None.
        show_default (bool, optional): _description_. Defaults to False.
        short (bool, optional): _description_. Defaults to True.

    Returns:
        Parameter: _description_
    """
    return Parameter(group=group, alias=alias, **kwargs)  # type: ignore
