from pathlib import Path
from evalio.cli.completions import DatasetOpt, PipelineOpt
from evalio.utils import print_warning
from tqdm.rich import tqdm
import numpy as np

from evalio.types import ImuMeasurement, LidarMeasurement
from evalio.rerun import RerunVis, VisArgs

from .parser import DatasetBuilder, PipelineBuilder, parse_config
from .writer import TrajectoryWriter, save_config, save_gt
from .stats import eval

from rich import print
from typing import Optional, Annotated
import typer


app = typer.Typer()


@app.command(no_args_is_help=True, name="run", help="Run pipelines on datasets")
def run_from_cli(
    config: Annotated[
        Optional[Path],
        typer.Option(
            "-c",
            "--config",
            help="Config file to load from",
            rich_help_panel="From config",
            show_default=False,
        ),
    ] = None,
    in_datasets: DatasetOpt = None,
    in_pipelines: PipelineOpt = None,
    in_out: Annotated[
        Optional[Path],
        typer.Option(
            "-o",
            "--output",
            help="Output directory to save results",
            rich_help_panel="Manual options",
            show_default=False,
        ),
    ] = None,
    length: Annotated[
        Optional[int],
        typer.Option(
            "-l",
            "--length",
            help="Number of scans to process for each dataset",
            rich_help_panel="Manual options",
            show_default=False,
        ),
    ] = None,
    visualize: Annotated[
        bool,
        typer.Option(
            "-v",
            "--visualize",
            help="Visualize the results via rerun",
            show_default=False,
        ),
    ] = False,
    show: Annotated[
        Optional[VisArgs],
        typer.Option(
            "-s",
            "--show",
            help="Show visualization options (m: map, i: image, s: scan, f: features). Automatically implies -v.",
            show_default=False,
            parser=VisArgs.parse,
        ),
    ] = None,
):
    if (in_pipelines or in_datasets or length) and config:
        raise typer.BadParameter(
            "Cannot specify both config and manual options", param_hint="run"
        )

    # Go through visualization options
    if show is None:
        vis_args = VisArgs(show=visualize)
    else:
        vis_args = show
    vis = RerunVis(vis_args)

    # Parse the config file if provided
    if config is not None:
        pipelines, datasets, out = parse_config(config)
        if out is None:
            out = Path("./evalio_results") / config.stem

    else:
        if in_pipelines is None:
            raise typer.BadParameter(
                "Must specify at least one pipeline", param_hint="run"
            )
        if in_datasets is None:
            raise typer.BadParameter(
                "Must specify at least one dataset", param_hint="run"
            )

        pipelines = PipelineBuilder.parse(in_pipelines)
        datasets = DatasetBuilder.parse(in_datasets)

        if length:
            for d in datasets:
                d.length = length

        if in_out is None:
            print_warning("Output directory not set. Defaulting to './evalio_results'")
            out = Path("./evalio_results")
        else:
            out = in_out

    if out.suffix == ".csv" and (len(pipelines) > 1 or len(datasets) > 1):
        raise typer.BadParameter(
            "Output must be a directory when running multiple experiments",
            param_hint="run",
        )

    run(pipelines, datasets, out, vis)


def plural(num: int, word: str) -> str:
    return f"{num} {word}{'s' if num > 1 else ''}"


def run(
    pipelines: list[PipelineBuilder],
    datasets: list[DatasetBuilder],
    output: Path,
    vis: RerunVis,
):
    print(
        f"Running {plural(len(pipelines), 'pipeline')} on {plural(len(datasets), 'dataset')} => {plural(len(pipelines) * len(datasets), 'experiment')}"
    )
    lengths = [
        min(d.length if d.length is not None else np.inf, len(d.build()))
        for d in datasets
    ]
    dtime = sum(le / d.dataset.lidar_params().rate for le, d in zip(lengths, datasets))  # type: ignore
    dtime *= len(pipelines)
    if dtime > 3600:
        print(f"Estimated time (if real-time): {dtime / 3600:.2f} hours")
    elif dtime > 60:
        print(f"Estimated time (if real-time): {dtime / 60:.2f} minutes")
    else:
        print(f"Estimated time (if real-time): {dtime:.2f} seconds")
    print(f"Output will be saved to {output}\n")
    save_config(pipelines, datasets, output)

    for dbuilder in datasets:
        save_gt(output, dbuilder)
        vis.new_recording(dbuilder.build(), pipelines)

        # Found how much we'll be iterating
        length = len(dbuilder.build().data_iter())
        if dbuilder.length is not None and dbuilder.length < length:
            length = dbuilder.length

        for pbuilder in pipelines:
            print(f"Running {pbuilder} on {dbuilder}")
            # Build everything
            dataset = dbuilder.build()
            pipe = pbuilder.build(dataset)
            writer = TrajectoryWriter(output, pbuilder, dbuilder)
            vis.new_pipe(pbuilder.name)

            # Run the pipeline
            loop = tqdm(total=length)
            for data in dbuilder.build():
                if isinstance(data, ImuMeasurement):
                    pipe.add_imu(data)
                elif isinstance(data, LidarMeasurement):  # pyright: ignore reportUnnecessaryIsInstance
                    features = pipe.add_lidar(data)
                    pose = pipe.pose()
                    writer.write(data.stamp, pose)

                    vis.log(data, features, pose, pipe)

                    loop.update()
                    if loop.n >= length:
                        loop.close()
                        break

            writer.close()

    eval([str(output)], False, "RTEt")
