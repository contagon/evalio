# from pathlib import Path
# from typing import Annotated, Any, Optional


# import evalio
# from evalio.datasets import set_data_dir
from cyclopts import App
from cyclopts.completion import detect_shell
from evalio.datasets import NewerCollege2020
from .completions import spec

# import typer apps
# from .dataset_manager import app as app_dl
# from .ls import app as app_ls
# from .run import app as app_run
# from .stats import app as app_stats

app = App(
    help="Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets",
    help_on_error=True,
    # help_formatter=spec,
    # rich_markup_mode="rich",
    # no_args_is_help=True,
    # pretty_exceptions_enable=False,
)
app.register_install_completion_command(add_to_startup=False)  # type: ignore

app["--help"].group = "Admin"
app["--version"].group = "Admin"
app["--install-completion"].group = "Admin"

app.command("evalio.cli.ls:ls")
app.command("evalio.cli.dataset_manager:dl")
app.command("evalio.cli.dataset_manager:rm")
app.command("evalio.cli.dataset_manager:filter")

app.command("evalio.cli.stats:evaluate_cli", name="stats")


@app.command(name="--show-completion", group="Admin")
def print_completion():
    """
    Print shell completion script.
    """
    comp = app.generate_completion()
    if detect_shell() == "zsh":
        comp += "compdef _evalio evalio"
    print(comp)


# app.add_typer(app_dl)
# app.add_typer(app_ls)
# app.add_typer(app_run)
# app.add_typer(app_stats)


# def version_callback(value: bool):
#     """
#     Show version and exit.
#     """
#     if value:
#         print(evalio.__version__)
#         raise typer.Exit()


# def data_callback(value: Optional[Path]):
#     """
#     Set the data directory.
#     """
#     if value is not None:
#         set_data_dir(value)


# def module_callback(value: Optional[list[str]]) -> list[Any]:
#     """
#     Set the module to use.
#     """
#     if value is not None:
#         for module in value:
#             evalio._register_custom_modules(module)

#     return []


# @app.callback()
# def global_options(
#     # Marking this as a str for now to get autocomplete to work,
#     # Once this fix is released (hasn't been as of 0.15.2), we can change it to a Path
#     # https://github.com/fastapi/typer/pull/1138
#     data_dir: Annotated[
#         Optional[Path],
#         typer.Option(
#             "-D",
#             "--data-dir",
#             help="Directory to store downloaded datasets.",
#             show_default=False,
#             rich_help_panel="Global options",
#             callback=data_callback,
#         ),
#     ] = None,
#     custom_modules: Annotated[
#         Optional[list[str]],
#         typer.Option(
#             "-M",
#             "--module",
#             help="Custom module to load (for custom datasets or pipelines). Can be used multiple times.",
#             show_default=False,
#             rich_help_panel="Global options",
#             callback=module_callback,
#         ),
#     ] = None,
#     version: Annotated[
#         bool,
#         typer.Option(
#             "--version",
#             "-V",
#             help="Show version and exit.",
#             is_eager=True,
#             show_default=False,
#             callback=version_callback,
#         ),
#     ] = False,
# ):
#     """
#     Global options for the evalio CLI.
#     """
#     pass


__all__ = [
    "app",
]

if __name__ == "__main__":
    app()
