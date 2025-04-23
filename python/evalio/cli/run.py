from pathlib import Path
from evalio.cli.completions import DatasetOpt, PipelineOpt
from evalio.utils import print_warning
from tqdm import tqdm

from evalio.types import ImuMeasurement, LidarMeasurement
from evalio.rerun import RerunVis

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
        Optional[str],
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
        Optional[str],
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
        int,
        typer.Option(
            "-v",
            "--visualize",
            count=True,
            help="Visualize results. Repeat up to 3 times for more detail",
            show_default=False,
        ),
    ] = 0,
):
    if (in_pipelines or in_datasets or length) and config:
        raise typer.BadParameter(
            "Cannot specify both config and manual options", param_hint="run"
        )

    vis = RerunVis(visualize)

    if config is not None:
        pipelines, datasets, out = parse_config(Path(config))
        if out is None:
            print_warning("Output directory not set. Defaulting to './evalio_results'")
            out = Path("./evalio_results")
        run(pipelines, datasets, out, vis)

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
            out = Path(in_out)

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
    print(f"Output will be saved to {output}\n")
    save_config(pipelines, datasets, output)

    for dbuilder in datasets:
        save_gt(output, dbuilder)

        for pbuilder in pipelines:
            print(f"Running {pbuilder} on {dbuilder}")
            # Build everything
            dataset = dbuilder.build()
            pipe = pbuilder.build(dataset)
            writer = TrajectoryWriter(output, pbuilder, dbuilder)

            # Initialize params
            first_scan_done = False
            data_iter = dataset.data_iter()
            length = len(data_iter)
            if dbuilder.length is not None and dbuilder.length < length:
                length = dbuilder.length
            loop = tqdm(total=length)

            # Run the pipeline
            for data in data_iter:
                if isinstance(data, ImuMeasurement):
                    pipe.add_imu(data)
                elif isinstance(data, LidarMeasurement):
                    features = pipe.add_lidar(data)
                    pose = pipe.pose()
                    writer.write(data.stamp, pose)

                    if not first_scan_done:
                        vis.new_recording(dataset)
                        first_scan_done = True

                    vis.log(data, features, pose, pipe)

                    loop.update()
                    if loop.n >= length:
                        loop.close()
                        break

            writer.close()

    eval([str(output)], False, "atet")
