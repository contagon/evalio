from pathlib import Path
from typing import Annotated, Callable, Optional, Sequence

import itertools

from evalio.utils import print_warning
from rich.table import Table
from rich.console import Console
from rich import box

from evalio.types import Trajectory
from evalio import stats

import numpy as np
import typer

import distinctipy

from joblib import Parallel, delayed

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
    reverse: bool,
    window_kind: stats.WindowKind,
    window_size: Optional[int | float],
    metric: stats.MetricKind,
    length: Optional[int],
    filter_method: Callable[[dict], bool],
    hide_name: bool = False,
    quiet: bool = False,
) -> Optional[list[dict]]:
    # Load all trajectories
    gt_list: list[Trajectory] = []
    all_trajs: list[Trajectory] = []
    for file_path in dir.glob("*.csv"):
        traj = Trajectory.from_experiment(file_path)
        if "gt" in traj.metadata:
            gt_list.append(traj)
        else:
            all_trajs.append(traj)

    assert len(gt_list) == 1, f"Found multiple ground truths in {dir}"
    gt_og = gt_list[0]

    # Setup visualization
    if visualize:
        try:
            import rerun as rr
        except Exception:
            print_warning("Rerun not found, visualization disabled")
            visualize = False

    rr = None
    convert = None
    colors = None
    if visualize:
        import rerun as rr
        from evalio.rerun import convert, GT_COLOR

        rr.init(
            str(dir),
            spawn=False,
        )
        rr.connect_grpc()
        rr.log(
            "gt",
            convert(gt_og, color=GT_COLOR),
            static=True,
        )

        # generate colors for visualization
        colors = distinctipy.get_colors(len(all_trajs) + 1)

    # Group into pipelines so we can compare keys
    # (other pipelines will have different keys)
    pipelines = set(traj.metadata["pipeline"] for traj in all_trajs)
    grouped_trajs: dict[str, list[Trajectory]] = {p: [] for p in pipelines}
    for traj in all_trajs:
        grouped_trajs[traj.metadata["pipeline"]].append(traj)

    # Compare keys in the same pipeline
    keys_to_print = ["pipeline"]
    for _, trajs in grouped_trajs.items():
        keys = dict_diff([traj.metadata for traj in trajs])
        if "name" in keys:
            keys.remove("name")
        keys_to_print += keys
    # These will all be pulled manually later
    if "total_elapsed" in keys_to_print:
        keys_to_print.remove("total_elapsed")
    if "max_step_elapsed" in keys_to_print:
        keys_to_print.remove("max_step_elapsed")
    if "length" in keys_to_print:
        keys_to_print.remove("length")

    # see if we should include the status
    if len(set(traj.metadata["status"] for traj in all_trajs)) > 1:
        keys_to_print.append("status")

    results: list[dict] = []
    index = 0
    for pipeline, trajs in grouped_trajs.items():
        # Iterate over each
        for traj in trajs:
            traj_aligned, gt_aligned = stats.align(traj, gt_og)
            if length is not None and len(traj_aligned) > length:
                traj_aligned.stamps = traj_aligned.stamps[:length]
                traj_aligned.poses = traj_aligned.poses[:length]
                gt_aligned.stamps = gt_aligned.stamps[:length]
                gt_aligned.poses = gt_aligned.poses[:length]
            ate = stats.ate(traj_aligned, gt_aligned).summarize(metric)
            rte = stats.rte(
                traj_aligned, gt_aligned, window_kind, window_size
            ).summarize(metric)
            hz = traj.metadata["total_elapsed"]
            if isinstance(hz, float):
                hz = len(traj) / hz
            r = {
                "name": traj.metadata["name"],
                "RTEt": rte.trans,
                "RTEr": rte.rot,
                "ATEt": ate.trans,
                "ATEr": ate.rot,
                "length": len(traj_aligned),
                "Hz": hz,
                "Max (s)": traj.metadata["max_step_elapsed"],
            }
            r.update({k: traj.metadata.get(k, "--") for k in keys_to_print})
            results.append(r)

            if (
                rr is not None
                and convert is not None
                and colors is not None
                and visualize
            ):
                rr.log(
                    traj.metadata["name"],
                    convert(traj_aligned, color=colors[index]),
                    static=True,
                )
                index += 1

    def key_func(x):
        val = x[sort]
        # sort floats separately to make sure NaNs are at the end
        if isinstance(val, float):
            return np.inf if np.isnan(val) else val
        else:
            return val

    try:
        results = [r for r in results if filter_method(r)]
    except Exception as e:
        print_warning(
            f"Error while running custom filter. Make sure it is valid python: {e}"
        )
        return
    results = sorted(results, key=key_func, reverse=reverse)
    if len(results) == 0:
        print_warning(f"No results found in {dir} after filtering.")
        return

    table = Table(
        title=str(dir),
        highlight=True,
        box=box.ROUNDED,
        min_width=len(str(dir)) + 5,
    )

    for r in results:
        if hide_name:
            r.pop("name", None)

    for key, val in results[0].items():
        table.add_column(key, justify="right" if isinstance(val, float) else "center")

    for result in results:
        row = [
            f"{item:.3f}" if isinstance(item, float) else str(item)
            for item in result.values()
        ]
        table.add_row(*row)

    if not quiet:
        print()
        Console().print(table)

    for r in results:
        r["dataset"] = gt_og.metadata["dataset"]
        r["sequence"] = gt_og.metadata["sequence"]

    return results


def _contains_dir(directory: Path) -> bool:
    return any(directory.is_dir() for directory in directory.glob("*"))


@app.command("stats", no_args_is_help=True)
def evaluate(
    directories: Annotated[
        list[str], typer.Argument(help="Directory of results to evaluate.")
    ],
    visualize: Annotated[
        bool, typer.Option("--visualize", "-v", help="Visualize results.")
    ] = False,
    # output options
    hide_name: Annotated[
        bool,
        typer.Option(
            "--hide-name",
            "-n",
            help="Show the name of the trajectory in the results.",
            rich_help_panel="Output options",
        ),
    ] = False,
    quiet: Annotated[
        bool,
        typer.Option(
            "--quiet",
            "-q",
            help="Don't print results to console.",
            rich_help_panel="Output options",
        ),
    ] = False,
    sort: Annotated[
        str,
        typer.Option(
            "-s",
            "--sort",
            help="Sort results by the name of a column.",
            rich_help_panel="Output options",
        ),
    ] = "RTEt",
    reverse: Annotated[
        bool,
        typer.Option(
            "--reverse",
            "-r",
            help="Reverse the sorting order. Defaults to False.",
            rich_help_panel="Output options",
        ),
    ] = False,
    # filtering options
    filter_str: Annotated[
        Optional[str],
        typer.Option(
            "--filter",
            "-f",
            help="Python expressions to filter results rows. 'True' rows will be kept. Example: --filter 'RTEt < 0.5'",
            rich_help_panel="Filtering options",
        ),
    ] = None,
    only_complete: Annotated[
        bool,
        typer.Option(
            "--only-complete",
            help="Only show results for trajectories that completed.",
            rich_help_panel="Filtering options",
        ),
    ] = False,
    only_incomplete: Annotated[
        bool,
        typer.Option(
            "--only-incomplete",
            help="Only show results for trajectories that did not finish.",
            rich_help_panel="Filtering options",
        ),
    ] = False,
    only_failed: Annotated[
        bool,
        typer.Option(
            "--only-failed",
            help="Only show results for trajectories that failed.",
            rich_help_panel="Filtering options",
        ),
    ] = False,
    # metric options
    window_size: Annotated[
        Optional[float],
        typer.Option(
            "-w",
            "--window-size",
            help="Window size for RTE. Defaults to 100 time steps for time windows, 10 meters for distance windows.",
            rich_help_panel="Metric options",
        ),
    ] = None,
    window_kind: Annotated[
        stats.WindowKind,
        typer.Option(
            "-k",
            "--window-kind",
            help="Kind of window to use for RTE. Defaults to time.",
            rich_help_panel="Metric options",
        ),
    ] = stats.WindowKind.time,
    metric: Annotated[
        stats.MetricKind,
        typer.Option(
            "--metric",
            "-m",
            help="Metric to use for ATE/RTE computation. Defaults to sse.",
            rich_help_panel="Metric options",
        ),
    ] = stats.MetricKind.sse,
    length: Annotated[
        Optional[int],
        typer.Option(
            "-l",
            "--length",
            help="Specify subset of trajectory to evaluate.",
            rich_help_panel="Metric options",
        ),
    ] = None,
) -> list[dict]:
    """
    Evaluate the results of experiments.
    """

    # Parse some of the options
    if sum([only_complete, only_incomplete, only_failed]) > 1:
        raise typer.BadParameter(
            "Can only use one of --only-complete, --only-incomplete, or --only-failed."
        )

    # Parse the filtering options
    if filter_str is None:
        filter_method = lambda r: True  # noqa: E731
    else:
        # TODO: Would be great to find a way around having to use eval here
        filter_method = lambda r: eval(  # noqa: E731
            filter_str,
            {"__builtins__": None},
            {"np": np, **r},
        )

    original_filter = filter_method
    if only_complete:
        filter_method = lambda r: original_filter(r) and r["status"] == "complete"  # noqa: E731
    elif only_incomplete:
        filter_method = lambda r: original_filter(r) and r["status"] == "--"  # noqa: E731
    elif only_failed:
        filter_method = lambda r: original_filter(r) and r["status"] == "fail"  # noqa: E731

    directories_path = [Path(d) for d in directories]

    if not quiet:
        match window_kind:
            case stats.WindowKind.distance:
                if window_size is None:
                    window_size = 10
                words = "meters"
            case stats.WindowKind.time:
                if window_size is None:
                    window_size = 200
                words = "time steps"
        c = Console()
        c.print(
            f"Evaluating RTE over a window of {window_size} {words}, using metric {metric}."
        )

    # Collect all bottom level directories
    bottom_level_dirs = []
    for directory in directories_path:
        for subdir in directory.glob("**/"):
            if not _contains_dir(subdir):
                bottom_level_dirs.append(subdir)

    # Compute them all in parallel
    results = Parallel(n_jobs=-2)(
        delayed(eval_dataset)(
            d,
            visualize,
            sort,
            reverse,
            window_kind,
            window_size,
            metric,
            length,
            filter_method,
            hide_name,
            quiet,
        )
        for d in bottom_level_dirs
    )
    results = [r for r in results if r is not None]

    return list(itertools.chain.from_iterable(results))
