import typer

# import functions
from .data_manager import dl, rm
from .ls import ls

# import typer apps
from .data_manager import app as app_dl
from .ls import app as app_ls

app = typer.Typer(
    help="Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets",
    rich_markup_mode="rich",
    no_args_is_help=True,
)

app.add_typer(app_dl)
app.add_typer(app_ls)

# eval "$(evalio2 --show-completion)"
__all__ = [
    "app",
    "dl",
    "rm",
    "ls",
]
