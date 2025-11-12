import multiprocessing
from pathlib import Path
from evalio._cpp.types import VisOption  # type: ignore
from evalio.cli.completions import DatasetOpt, PipelineOpt
from evalio.types.base import Trajectory
from evalio.utils import print_warning
from tqdm.rich import tqdm
import yaml

from evalio import datasets as ds, pipelines as pl, types as ty
from evalio.rerun import RerunVis

# from .stats import evaluate

from rich import print
from typing import Optional, Annotated
import typer

from time import time


app = typer.Typer()


def parse_vis(opts: str) -> set[VisOption]:
    out = set[VisOption]()
    for o in opts:
        match o:
            case "m":
                out.add(VisOption.MAP)
            case "i":
                out.add(VisOption.IMAGE)
            case "s":
                out.add(VisOption.SCAN)
            case "f":
                out.add(VisOption.FEATURES)
            case _:
                raise typer.BadParameter(f"Unknown visualization option {o}")

    return out


@app.command(no_args_is_help=True, name="run", help="Run pipelines on datasets")
def run_from_cli(
    # Config file
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
    # Manual options
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
    # misc options
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
        Optional[set[VisOption]],
        typer.Option(
            "-s",
            "--show",
            help="Show visualization options (m: map, i: image, s: scan, f: features). Automatically implies -v.",
            show_default=False,
            parser=parse_vis,
        ),
    ] = None,
    rerun_failed: Annotated[
        bool,
        typer.Option(
            "--rerun-failed",
            help="Rerun failed experiments. If not set, will skip previously failed experiments.",
            show_default=False,
        ),
    ] = False,
):
    if (in_pipelines or in_datasets or length) and config:
        raise typer.BadParameter(
            "Cannot specify both config and manual options", param_hint="run"
        )

    # ------------------------- Parse Config file ------------------------- #
    if config is not None:
        # load from yaml
        with open(config, "r") as f:
            try:
                Loader = yaml.CSafeLoader
            except Exception as _:
                print_warning(
                    "Failed to import yaml.CSafeLoader, trying yaml.SafeLoader"
                )
                Loader = yaml.SafeLoader

            params = yaml.load(f, Loader=Loader)

        if "datasets" not in params:
            raise typer.BadParameter(
                "No datasets specified in config", param_hint="run"
            )
        if "pipelines" not in params:
            raise typer.BadParameter(
                "No pipelines specified in config", param_hint="run"
            )

        datasets = ds.parse_config(params.get("datasets", None))
        pipelines = pl.parse_config(params.get("pipelines", None))

        out = (
            params["output_dir"]
            if "output_dir" in params
            else Path("./evalio_results") / config.stem
        )

    # ------------------------- Parse manual options ------------------------- #
    else:
        if in_pipelines is None:
            raise typer.BadParameter(
                "Must specify at least one pipeline", param_hint="run"
            )
        if in_datasets is None:
            raise typer.BadParameter(
                "Must specify at least one dataset", param_hint="run"
            )

        if length is not None:
            temp_datasets: list[ds.DatasetConfig] = [
                {"name": d, "length": length} for d in in_datasets
            ]
        else:
            temp_datasets = [{"name": d} for d in in_datasets]

        pipelines = pl.parse_config(in_pipelines)
        datasets = ds.parse_config(temp_datasets)

        if in_out is None:
            print_warning("Output directory not set. Defaulting to './evalio_results'")
            out = Path("./evalio_results")
        else:
            out = in_out

    # ------------------------- Miscellaneous ------------------------- #
    # error out if either is wrong
    if isinstance(datasets, ds.DatasetConfigError):
        raise typer.BadParameter(
            f"Error in datasets config: {datasets}", param_hint="run"
        )
    if isinstance(pipelines, pl.PipelineConfigError):
        raise typer.BadParameter(
            f"Error in pipelines config: {pipelines}", param_hint="run"
        )

    if out.suffix == ".csv" and (len(pipelines) > 1 or len(datasets) > 1):
        raise typer.BadParameter(
            "Output must be a directory when running multiple experiments",
            param_hint="run",
        )

    # make sure all datasets are downloaded
    for d, _ in datasets:
        d._fail_not_downloaded()

    # parse all of the lengths
    datasets = [
        (s, len(s) if length is None else min(len(s), length)) for s, length in datasets
    ]

    print(
        f"Running {plural(len(datasets), 'dataset')} => {plural(len(pipelines) * len(datasets), 'experiment')}"
    )
    dtime = sum(le / d.lidar_params().rate for d, le in datasets)
    dtime *= len(pipelines)
    if dtime > 3600:
        print(f"Estimated time (if real-time): {dtime / 3600:.2f} hours")
    elif dtime > 60:
        print(f"Estimated time (if real-time): {dtime / 60:.2f} minutes")
    else:
        print(f"Estimated time (if real-time): {dtime:.2f} seconds")
    print(f"Output will be saved to {out}\n")

    # Go through visualization options
    vis_args: Optional[set[VisOption]]
    if show is None:
        if visualize:
            vis_args = set()
        else:
            vis_args = None
    else:
        vis_args = show
    vis = RerunVis(vis_args, [p[0] for p in pipelines])

    # save_config(pipelines, datasets, out)

    # ------------------------- Convert to experiments ------------------------- #
    experiments = [
        ty.Experiment(
            name=name,
            sequence=sequence,
            sequence_length=length,
            pipeline=pipeline,
            pipeline_version=pipeline.version(),
            pipeline_params=params,
            file=out / sequence.full_name / f"{name}.csv",
        )
        for sequence, length in datasets
        for name, pipeline, params in pipelines
    ]

    run(experiments, vis, rerun_failed)


def plural(num: int, word: str) -> str:
    return f"{num} {word}{'s' if num > 1 else ''}"


def run(
    experiments: list[ty.Experiment],
    vis: RerunVis,
    rerun_failed: bool,
):
    # Make sure everything is in the experiments that we need
    len_before = len(experiments)
    experiments = [
        exp
        for exp in experiments
        if isinstance(exp.sequence, ds.Dataset)
        and not isinstance(exp.pipeline, str)
        and exp.file is not None
    ]
    if len(experiments) < len_before:
        print_warning(
            f"Some experiments were invalid and will be skipped ({len_before - len(experiments)} out of {len_before})"
        )

    prev_dataset = None
    for exp in experiments:
        # For the type checker
        if (
            not isinstance(exp.sequence, ds.Dataset)
            or isinstance(exp.pipeline, str)
            or exp.file is None
        ):
            continue

        # save ground truth if we haven't already
        if not (gt_file := exp.file.parent / "gt.csv").exists():
            exp.sequence.ground_truth().to_file(gt_file)

        # Figure out the status of the experiment
        traj = ty.Trajectory.from_file(exp.file)
        if isinstance(traj, ty.Trajectory) and isinstance(traj.metadata, ty.Experiment):
            # If the sequence length has changed, mark as started
            if traj.metadata.sequence_length != exp.sequence_length:
                status = ty.ExperimentStatus.Started
            else:
                status = traj.metadata.status

        else:
            status = ty.ExperimentStatus.NotRun

        # Do something based on the status
        info = f"{exp.name} on {exp.sequence}"
        match status:
            case ty.ExperimentStatus.Complete:
                print(f"Skipping {info}, already finished")
                continue
            case ty.ExperimentStatus.Fail:
                if rerun_failed:
                    print(f"Rerunning {info}, previously failed")
                else:
                    print(f"Skipping {info}, previously failed")
                    continue
            case ty.ExperimentStatus.Started:
                print(f"Overwriting {info}")
            case ty.ExperimentStatus.NotRun:
                print(f"Running {info}")

        # start vis if needed
        if prev_dataset != exp.sequence:
            prev_dataset = exp.sequence
            vis.new_dataset(exp.sequence)

        # Run the pipeline in a different process so we can recover from segfaults
        process = multiprocessing.Process(target=run_single, args=(exp, vis))
        process.start()
        process.join()
        exitcode = process.exitcode
        process.close()

        # If it failed, mark the status as failed
        if exitcode != 0:
            exp.status = ty.ExperimentStatus.Fail
            traj = ty.Trajectory.from_file(exp.file)
            if isinstance(traj, ty.Trajectory) and isinstance(
                traj.metadata, ty.Experiment
            ):
                traj.metadata = exp
                traj.to_file()
            else:
                Trajectory(metadata=exp).to_file()

    # if len(experiments) > 1:
    #     if (file := experiments[0].file) is not None:
    #         evaluate([str(file.parent)])


def run_single(
    exp: ty.Experiment,
    vis: RerunVis,
):
    # Build everything
    output = exp.setup()
    if isinstance(output, ds.DatasetConfigError | pl.PipelineConfigError):
        print_warning(f"Error setting up experiment {exp.name}: {output}")
        return
    pipe, dataset = output
    pipe.set_visualizing(vis.args)
    exp.status = ty.ExperimentStatus.Started
    traj = ty.Trajectory(metadata=exp)
    traj.open()
    vis.new_pipe(exp.name, len(pipe.map()))

    time_running = 0.0
    time_max = 0.0
    time_total = 0.0

    loop = tqdm(total=exp.sequence_length)
    for data in dataset:
        if isinstance(data, ty.ImuMeasurement):
            start = time()
            pipe.add_imu(data)
            time_running += time() - start
        elif isinstance(data, ty.LidarMeasurement):  # type: ignore
            start = time()
            pipe.add_lidar(data)
            time_running += time() - start

            time_total += time_running
            if time_running > time_max:
                time_max = time_running
            time_running = 0.0

            vis.log_scan(data)

            loop.update()

        # Save and visualize any new information
        for stamp, pose in pipe.saved_estimates():
            traj.append(stamp, pose)
            vis.log_pose(stamp, pose)
        for stamp, features in pipe.saved_features_matrix():
            vis.log_features(stamp, features)
        for stamp, map in pipe.saved_maps_matrix():
            vis.log_map(stamp, map)

        if loop.n >= exp.sequence_length:
            loop.close()
            break

    loop.close()
    traj.metadata.status = ty.ExperimentStatus.Complete
    traj.metadata.total_elapsed = time_total
    traj.metadata.max_elapsed = time_max
    traj.rewrite()
    traj.close()
