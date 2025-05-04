from pathlib import Path
from typing import Annotated, Optional, Sequence

from rich.table import Table
from rich.console import Console
from rich import box

from evalio.types import Trajectory, MetricKind

import typer


app = typer.Typer()


def dict_diff(dicts: Sequence[dict]) -> list[str]:
    """Compute which values are different between a list of dictionaries.

    Assumes each dictionary has the same keys.

    Args:
        dicts (Sequence[dict]): List of dictionaries to compare.

    Returns:
        list[str]: Keys that don't have identical values between all dictionaries.
    """

    # quick sanity check
    size = len(dicts[0])
    for d in dicts:
        assert len(d) == size

    # compare all dictionaries to find varying keys
    diff = []
    for k in dicts[0].keys():
        if any(d[k] != dicts[0][k] for d in dicts):
            diff.append(k)

    return diff


def eval_dataset(
    dir: Path,
    visualize: bool,
    sort: Optional[str],
    window_size: int,
    metric: MetricKind,
    length: Optional[int],
):
    # Load all trajectories
    trajectories = []
    for file_path in dir.glob("*.csv"):
        traj = Trajectory.from_experiment(file_path)
        trajectories.append(traj)

    gt_list: list[Trajectory] = []
    trajs: list[Trajectory] = []
    for t in trajectories:
        (gt_list if "gt" in t.metadata else trajs).append(t)

    assert len(gt_list) == 1, f"Found multiple ground truths in {dir}"
    gt_og = gt_list[0]

    # Setup visualization
    if visualize:
        try:
            import rerun as rr
        except Exception:
            print("Rerun not found, visualization disabled")
            visualize = False

    rr = None
    convert = None
    if visualize:
        import rerun as rr
        from evalio.rerun import convert

        rr.init(
            str(dir),
            spawn=False,
        )
        rr.connect_grpc()
        rr.log(
            "gt",
            convert(gt_og, color=[0, 0, 255]),
            static=True,
        )

    # Group into pipelines so we can compare keys
    # (other pipelines will have different keys)
    pipelines = set(traj.metadata["pipeline"] for traj in trajs)
    grouped_trajs: dict[str, list[Trajectory]] = {p: [] for p in pipelines}
    for traj in trajs:
        grouped_trajs[traj.metadata["pipeline"]].append(traj)

    # Find all keys that were different
    keys_to_print = ["pipeline"]
    for _, trajs in grouped_trajs.items():
        keys = dict_diff([traj.metadata for traj in trajs])
        if len(keys) > 0:
            keys.remove("name")
            keys_to_print += keys

    results = []
    for pipeline, trajs in grouped_trajs.items():
        # Iterate over each
        for traj in trajs:
            traj_aligned, gt_aligned = Trajectory.align(traj, gt_og)
            ate = Trajectory.ate(traj_aligned, gt_aligned).summarize(metric)
            rte = Trajectory.rte(traj_aligned, gt_aligned, window_size).summarize(
                metric
            )
            r = {
                "name": traj.metadata["name"],
                "RTEt": rte.trans,
                "RTEr": rte.rot,
                "ATEt": ate.trans,
                "ATEr": ate.rot,
                "length": len(traj_aligned),
            }
            r.update({k: traj.metadata.get(k, "--") for k in keys_to_print})
            results.append(r)

            if rr is not None and convert is not None and visualize:
                rr.log(
                    traj.metadata["name"],
                    convert(traj),
                    static=True,
                )

    if sort is not None:
        results = sorted(results, key=lambda x: x[sort])

    table = Table(
        title=str(dir),
        highlight=True,
        box=box.ROUNDED,
        min_width=len(str(dir)) + 5,
    )

    for key, val in results[0].items():
        table.add_column(key, justify="right" if isinstance(val, float) else "center")

    for result in results:
        row = [
            f"{item:.3f}" if isinstance(item, float) else str(item)
            for item in result.values()
        ]
        table.add_row(*row)

    Console().print(table)
    print()


def _contains_dir(directory: Path) -> bool:
    return any(directory.is_dir() for directory in directory.glob("*"))


@app.command("stats", no_args_is_help=True)
def eval(
    directories: Annotated[
        list[str], typer.Argument(help="Directory of results to evaluate.")
    ],
    visualize: Annotated[
        bool, typer.Option("--visualize", "-v", help="Visualize results.")
    ] = False,
    sort: Annotated[
        Optional[str],
        typer.Option("-s", "--sort", help="Sort results by either [atet|ater]"),
    ] = None,
    window: Annotated[
        int,
        typer.Option(
            "-w", "--window", help="Window size for RTE. Defaults to 100 time-steps."
        ),
    ] = 100,
    metric: Annotated[
        MetricKind,
        typer.Option(
            "--metric",
            "-m",
            help="Metric to use for ATE/RTE computation. Defaults to sse.",
        ),
    ] = MetricKind.sse,
    length: Annotated[
        Optional[int],
        typer.Option(
            "-l", "--length", help="Specify subset of trajectory to evaluate."
        ),
    ] = None,
):
    """
    Evaluate the results of experiments.
    """

    directories_path = [Path(d) for d in directories]

    c = Console()
    c.print(f"Evaluating RTE over a window of size {window}, using metric {metric}.")
    c.print()

    # Collect all bottom level directories
    bottom_level_dirs = []
    for directory in directories_path:
        for subdir in directory.glob("**/"):
            if not _contains_dir(subdir):
                bottom_level_dirs.append(subdir)

    for d in bottom_level_dirs:
        eval_dataset(d, visualize, sort, window, metric, length)
