from inspect import isclass
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, TypeAlias, Literal

from cyclopts import Parameter
from cyclopts.help import DefaultFormatter, ColumnSpec, HelpEntry, TableSpec
from evalio import datasets as ds, pipelines as pl

from enum import Enum

from typing import Any, Union, get_args, get_origin


# ------------------------- Prettier Helper Pages ------------------------- #
# Define custom column renderers
def names_long(entry: HelpEntry) -> str:
    """Combine parameter names and shorts."""
    if not entry.names:
        return ""
    # If just the one, use
    if len(entry.names) == 1:
        return entry.names[0]
    # If multiples, skip the first (usually ALL_CAPS)
    if entry.names[0].isupper():
        names = entry.names[1:]
    else:
        names = entry.names

    return " ".join(names).strip()


def names_short(entry: HelpEntry) -> str:
    """Combine parameter names and shorts."""
    shorts = " ".join(entry.shorts) if entry.shorts else ""
    return shorts.strip()


def render_type(type_: Any) -> str:
    """Show the parameter type."""
    from cyclopts.annotations import get_hint_name  # type: ignore

    if type_ is bool:
        return ""
    elif type_ is str:
        return "text"
    elif type_ is int:
        return "int"
    elif type_ is float:
        return "float"
    elif type_ is Path:
        return "path"
    elif type_ is None:
        return ""
    elif (origin := get_origin(type_)) is Literal:
        args = get_args(type_)
        if len(args) > 5:
            return "text"
        return "|".join(str(a) for a in args)
    # elif type_ is None:
    # return "NONE"
    elif origin is Union:
        # handle Optional
        args = get_args(type_)
        if len(args) == 2 and type(None) in args:
            type_ = args[0] if args[1] is type(None) else args[1]
            return render_type(type_)
        else:
            return " | ".join(render_type(t) for t in args)
    elif origin is list:
        return render_type(get_args(type_)[0])
    elif isclass(type_) and issubclass(type_, Enum):
        return "|".join(e.name for e in type_)

    print(type_)
    return get_hint_name(type_) if type_ else ""


# Create custom columns
spec = DefaultFormatter(
    table_spec=TableSpec(show_header=False),
    column_specs=(
        ColumnSpec(
            renderer=lambda e: "*" if e.required else " ",  # type: ignore
            header="",
            width=2,
            style="red bold",
        ),
        ColumnSpec(
            renderer=names_long,
            style="cyan",
        ),
        ColumnSpec(
            renderer=names_short,
            style="green",
            max_width=30,
        ),
        ColumnSpec(
            renderer=lambda e: render_type(e.type),  # type: ignore
            style="yellow",
            justify="center",
        ),
        ColumnSpec(
            renderer="description",  # Use attribute name
            overflow="fold",
        ),
    ),
)


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
