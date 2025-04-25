from .parser import DatasetBuilder
import typer
from typing import Annotated
from .completions import DatasetArg


app = typer.Typer()


@app.command(no_args_is_help=True)
def dl(datasets: DatasetArg) -> None:
    """
    Download datasets
    """
    # parse all datasets
    valid_datasets = DatasetBuilder.parse(datasets)

    # Check if already downloaded
    to_download = []
    for builder in valid_datasets:
        if builder.is_downloaded():
            print(f"Skipping download for {builder}, already exists")
        else:
            to_download.append(builder)

    if len(to_download) == 0:
        print("Nothing to download, finishing")
        return

    # download each dataset
    print("Will download: ")
    for builder in to_download:
        print(f"  {builder}")
    print()

    for builder in to_download:
        print(f"---------- Beginning {builder} ----------")
        try:
            builder.download()
        except Exception as e:
            print(f"Error downloading {builder}\n: {e}")
        print(f"---------- Finished {builder} ----------")


@app.command(no_args_is_help=True)
def rm(
    datasets: DatasetArg,
    force: Annotated[
        bool,
        typer.Option(
            "--force",
            "-f",
            prompt="Are you sure you want to delete these datasets?",
            help="Force deletion without confirmation",
        ),
    ] = False,
):
    """
    Remove dataset(s)

    If --force is not used, will ask for confirmation.
    """
    # parse all datasets
    to_remove = DatasetBuilder.parse(datasets)

    print("Will remove: ")
    for builder in to_remove:
        print(f"  {builder}")
    print()

    for builder in to_remove:
        print(f"---------- Beginning {builder} ----------")
        try:
            print(f"Removing from {builder.dataset.folder}")
            for f in builder.dataset.files():
                print(f"  Removing {f}")
                (builder.dataset.folder / f).unlink()
        except Exception as e:
            print(f"Error removing {builder}\n: {e}")
        print(f"---------- Finished {builder} ----------")
