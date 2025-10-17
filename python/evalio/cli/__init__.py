# from pathlib import Path
# from typing import Annotated, Any, Optional


# import evalio
# from evalio.datasets import set_data_dir
from pathlib import Path
from typing import Annotated, Optional
from cyclopts import App, Group, Parameter
from cyclopts.completion import detect_shell
from .completions import spec

# import typer apps
# from .dataset_manager import app as app_dl
# from .ls import app as app_ls
# from .run import app as app_run
# from .stats import app as app_stats

app = App(
    help="Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets",
    help_on_error=True,
    help_formatter=spec,
    default_parameter=Parameter(negative="", show_default=False),
    # group="TESTING",
    group_parameters="Options",
    group_commands=Group("Commands", sort_key=1),
)
app.register_install_completion_command(add_to_startup=True)  # type: ignore

og = Group("Options", sort_key=0)

app["--help"].group = og
app["--version"].group = og
app["--install-completion"].group = og

app.command("evalio.cli.ls:ls")

app.command("evalio.cli.dataset_manager:dl")
app.command("evalio.cli.dataset_manager:rm")
app.command("evalio.cli.dataset_manager:filter")

app.command("evalio.cli.stats:evaluate_cli", name="stats")

app.command("evalio.cli.run:run_from_cli", name="run")


@app.command(name="--show-completion", group="Options")
def print_completion():
    """
    Print shell completion script.
    """
    comp = app.generate_completion()
    if detect_shell() == "zsh":
        comp += "compdef _evalio evalio"
    print(comp)


@app.meta.default
def global_options(
    *tokens: Annotated[str, Parameter(show=False, allow_leading_hyphen=True)],
    module: Annotated[Optional[list[str]], Parameter(alias="-M")] = None,
    data_dir: Annotated[Optional[Path], Parameter(alias="-D")] = None,
):
    """_summary_

    Args:
        module (list[str]): Custom module to import. Can be repeated.
        data_dir (Optional[Path]): Directory to store downloaded datasets.
    """

    if module is not None:
        from evalio import _register_custom_modules

        for m in module:
            _register_custom_modules(m)

    if data_dir is not None:
        from evalio.datasets import set_data_dir

        set_data_dir(data_dir)


# TODO: Disabling this is weird! Breaks help output
# app.meta()


# def version_callback(value: bool):
#     """
#     Show version and exit.
#     """
#     if value:
#         print(evalio.__version__)
#         raise typer.Exit()


__all__ = [
    "app",
]

if __name__ == "__main__":
    app()
