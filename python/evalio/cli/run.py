import multiprocessing
from pathlib import Path
from cyclopts import Group, Token, ValidationError
from cyclopts import Parameter
from evalio.cli.types import DataSequence, Pipeline, Param
from evalio.types import Trajectory
from evalio.utils import print_warning
from tqdm.rich import tqdm
import yaml

from evalio import datasets as ds, pipelines as pl, types as ty
from evalio.rerun import RerunVis, VisOptions

from rich import print
from typing import TYPE_CHECKING, Literal, Optional, Sequence, Annotated

from time import time

if TYPE_CHECKING:
    VisStr = str
else:
    # Use literal instead of enum for cyclopts to parse as value instead of name
    VisStr = Literal[(tuple(v.value for v in VisOptions))]


def vis_convert(type_: type, tokens: Sequence[Token]) -> Optional[list[str]]:
    """Custom converter to split strings into individual characters."""
    out: list[str] = []
    options = [v.value for v in VisOptions]
    for t in tokens:
        if t.value in options:
            out.append(t.value)
        elif len(t.value) > 1:
            for c in t.value:
                if c in options:
                    out.append(c)

    return out


# shorten things for annotation
Ann = Annotated
Opt = Optional
Par = Parameter

# groups
cg = Group("Config", sort_key=0)
mg = Group("Manual", sort_key=1)
og = Group("Misc", sort_key=2)


def run_from_cli(
    # Config file
    config: Ann[Opt[Path], Par(alias="-c", group=cg)] = None,
    # Manual options
    datasets: Ann[Opt[list[DataSequence]], Par(alias="-d", group=mg)] = None,
    pipelines: Ann[Opt[list[Pipeline]], Par(alias="-p", group=mg)] = None,
    out: Ann[Opt[Path], Par(alias="-o", group=mg)] = None,
    length: Ann[Opt[int], Par(alias="-l", group=mg)] = None,
    # misc options
    rerun_failed: Ann[bool, Param(group=og)] = False,
    visualize: Ann[
        Opt[list[VisStr]],
        Par(
            alias="-v",
            group=og,
            consume_multiple=True,
            converter=vis_convert,
        ),
    ] = None,
):
    """Run lidar-inertial odometry experiments.

    Args:
        config (Path): Path to the config file.
        datasets (Dataset): Input datasets. May be repeated.
        pipelines (Pipeline): Input pipelines. May be repeated.
        out (Path): Output directory to save results.
        length (int): Number of scans to process for each dataset.
        visualize (list[VisOptions]): Visualization options. If just '-v', will show trajectory. Add more via '-v misf' (m: map, i: image, s: scan, f: features).
        rerun_failed (bool): Rerun failed experiments. By default, failed experiments are skipped.
    """

    if (pipelines or datasets or length) and config:
        raise ValueError("Cannot specify both config and manual options")

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
            raise ValueError("No datasets specified in config")
        if "pipelines" not in params:
            raise ValueError("No pipelines specified in config")

        run_datasets = ds.parse_config(params.get("datasets", None))
        run_pipelines = pl.parse_config(params.get("pipelines", None))

        run_out = (
            params["output_dir"]
            if "output_dir" in params
            else Path("./evalio_results") / config.stem
        )

    # ------------------------- Parse manual options ------------------------- #
    else:
        if pipelines is None:
            raise ValueError("Must specify at least one pipeline")
        if datasets is None:
            raise ValidationError(msg="Must specify at least one dataset")

        if length is not None:
            temp_datasets: list[ds.DatasetConfig] = [
                {"name": d, "length": length} for d in datasets
            ]
        else:
            temp_datasets = [{"name": d} for d in datasets]

        run_pipelines = pl.parse_config(pipelines)
        run_datasets = ds.parse_config(temp_datasets)

        if out is None:
            print_warning("Output directory not set. Defaulting to './evalio_results'")
            run_out = Path("./evalio_results")
        else:
            run_out = out

    # ------------------------- Miscellaneous ------------------------- #
    # error out if either is wrong
    if isinstance(run_datasets, ds.DatasetConfigError):
        raise ValueError(f"Error in datasets config: {run_datasets}")
    if isinstance(run_pipelines, pl.PipelineConfigError):
        raise ValueError(f"Error in pipelines config: {run_pipelines}")

    if run_out.suffix == ".csv" and (len(run_pipelines) > 1 or len(run_datasets) > 1):
        raise ValueError("Output must be a directory when running multiple experiments")

    print(
        f"Running {plural(len(run_datasets), 'dataset')} => {plural(len(run_pipelines) * len(run_datasets), 'experiment')}"
    )
    dtime = sum(le / d.lidar_params().rate for d, le in run_datasets)
    dtime *= len(run_pipelines)
    if dtime > 3600:
        print(f"Estimated time (if real-time): {dtime / 3600:.2f} hours")
    elif dtime > 60:
        print(f"Estimated time (if real-time): {dtime / 60:.2f} minutes")
    else:
        print(f"Estimated time (if real-time): {dtime:.2f} seconds")
    print(f"Output will be saved to {run_out}\n")

    # Go through visualization options
    vis_args = None
    if visualize is not None:
        vis_args = [VisOptions(v) for v in visualize]

    vis = RerunVis(vis_args, [p[0] for p in run_pipelines])

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
            file=run_out / sequence.full_name / f"{name}.csv",
        )
        for sequence, length in run_datasets
        for name, pipeline, params in run_pipelines
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
    exp.status = ty.ExperimentStatus.Started
    traj = ty.Trajectory(metadata=exp)
    traj.open(exp.file)
    vis.new_pipe(exp.name)

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
            features = pipe.add_lidar(data)
            pose = pipe.pose()
            time_running += time() - start

            time_total += time_running
            if time_running > time_max:
                time_max = time_running
            time_running = 0.0

            traj.append(data.stamp, pose)
            vis.log(data, features, pose, pipe)

            loop.update()
            if loop.n >= exp.sequence_length:
                loop.close()
                break

    loop.close()
    traj.metadata.status = ty.ExperimentStatus.Complete
    traj.metadata.total_elapsed = time_total
    traj.metadata.max_elapsed = time_max
    traj.rewrite()
    traj.close()
