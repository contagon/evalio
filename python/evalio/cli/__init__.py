from inspect import isclass
from pathlib import Path
from cyclopts import App, Group, Parameter
from cyclopts.completion import detect_shell
from cyclopts.help import DefaultFormatter, ColumnSpec, HelpEntry, PanelSpec, TableSpec
from enum import Enum
from rich.console import Console, ConsoleOptions
from typing import Any, Union, get_args, get_origin, Literal, Annotated, Optional


# ------------------------- Prettier Helper Pages ------------------------- #
# Define custom column renderers
def names_long(entry: HelpEntry) -> str:
    """Combine parameter names and shorts."""
    if not entry.names:
        return ""
    # If positional with single name, just return it
    if len(entry.names) == 1:
        return entry.names[0]
    # If multiples, skip ALL_CAPS
    if entry.names[0].isupper():
        names = entry.names[1:]
    else:
        names = entry.names

    return " ".join(names).strip()


def render_type(type_: Any) -> str:
    """Show the parameter type."""
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

    raise ValueError(f"Unsupported type for rendering: {type_}")


def columns(
    console: Console, options: ConsoleOptions, entries: list[HelpEntry]
) -> list[ColumnSpec]:
    columns: list[ColumnSpec] = []

    if any(e.required for e in entries):
        columns.append(
            ColumnSpec(
                renderer=lambda e: "*" if e.required else " ",  # type: ignore
                width=1,
                style="red bold",
            )
        )

    columns.extend(
        [
            ColumnSpec(
                renderer=names_long,
                style="cyan",
            ),
            ColumnSpec(
                renderer=lambda e: " ".join(e.shorts).strip() if e.shorts else "",  # type: ignore
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
        ]
    )

    return columns


# Create custom columns
spec = DefaultFormatter(
    table_spec=TableSpec(show_header=False),
    column_specs=columns,  # type: ignore
    panel_spec=PanelSpec(border_style="bright_black"),
)


# ------------------------- Make CLI App ------------------------- #
mg = Group("Misc", sort_key=1)
gg = Group("Global Options", sort_key=100)

app = App(
    help="Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets",
    help_formatter=spec,
    help_on_error=True,
    default_parameter=Parameter(negative=""),
    version_flags=[],
)

# Register commands
app.register_install_completion_command(add_to_startup=True)  # type: ignore
app.command("evalio.cli.ls:ls")
app.command("evalio.cli.dataset_manager:dl")
app.command("evalio.cli.dataset_manager:rm")
app.command("evalio.cli.dataset_manager:filter")
app.command("evalio.cli.stats:evaluate_cli", name="stats")
app.command("evalio.cli.run:run_from_cli", name="run")

# Assign groups
app["--install-completion"].group = mg
app["--help"].group = gg

for c in app:
    if c in ["--help", "-h"]:
        continue
    app[c]["--help"].group = gg


@app.command(name="--show-completion", group=mg)
def show_completion():
    """
    Show shell completion script.
    """
    comp = app.generate_completion()
    # zsh needs an extra line
    if detect_shell() == "zsh":
        comp += "compdef _evalio evalio"
        # Fix wildcard completions
        comp = comp.replace("/*", "/\\*")
    print(comp)


@app.command(name="--version", alias="-V", group=mg)
def version():
    """
    Show version and exit.
    """
    import evalio

    print(evalio.__version__)


@app.meta.default
def global_options(
    *tokens: Annotated[str, Parameter(show=False, allow_leading_hyphen=True)],
    module: Annotated[Optional[list[str]], Parameter(alias="-M", group=gg)] = None,
    data_dir: Annotated[Optional[Path], Parameter(alias="-D", group=gg)] = None,
):
    """Define a number of global options that are set before any command is run.

    Args:
        module (list[str]): Custom module to import. Can be repeated.
        data_dir (Optional[Path]): Directory to store downloaded datasets.
    """
    # Register custom modules
    if module is not None:
        from evalio import _register_custom_modules

        for m in module:
            _register_custom_modules(m)

    # Set data directory
    if data_dir is not None:
        from evalio.datasets import set_data_dir

        set_data_dir(data_dir)

    app(tokens)


def launch():
    app.meta()


__all__ = [
    "app",
]
