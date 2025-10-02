from copy import copy
from pathlib import Path
from typing import Annotated, Any, Callable, Optional, cast

from evalio.types.base import Trajectory
import polars as pl
import itertools

from evalio.utils import print_warning
from rich.table import Table
from rich.console import Console
from rich import box

from evalio import types as ty, stats

import numpy as np
import typer

import distinctipy

from joblib import Parallel, delayed

app = typer.Typer()


def eval_dataset(
    dir: Path,
    visualize: bool,
    windows: list[stats.WindowKind],
    metric: stats.MetricKind,
    length: Optional[int],
) -> Optional[list[dict[str, Any]]]:
    # Load all trajectories
    gt_og: Optional[ty.Trajectory[ty.GroundTruth]] = None
    all_trajs: list[ty.Trajectory[ty.Experiment]] = []
    for file_path in dir.glob("*.csv"):
        traj = ty.Trajectory.from_file(file_path)
        if not isinstance(traj, ty.Trajectory):
            print_warning(f"Could not load trajectory from {file_path}, skipping.")
            continue
        elif isinstance(traj.metadata, ty.GroundTruth):
            if gt_og is not None:
                print_warning(f"Multiple ground truths found in {dir}, skipping.")
                continue
            gt_og = cast(ty.Trajectory[ty.GroundTruth], traj)
        elif isinstance(traj.metadata, ty.Experiment):
            all_trajs.append(cast(ty.Trajectory[ty.Experiment], traj))

    if gt_og is None:
        print_warning(f"No ground truth found in {dir}, skipping.")
        return None

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

    # Iterate over each
    results: list[dict[str, Any]] = []
    for index, traj in enumerate(all_trajs):
        r = traj.metadata.to_dict()
        # flatten pipeline params
        r.update(r["pipeline_params"])
        del r["pipeline_params"]

        # add metrics
        gt_aligned = Trajectory(
            stamps=[copy(s) for s in gt_og.stamps],
            poses=[copy(p) for p in gt_og.poses],
        )
        stats.align(traj, gt_aligned, in_place=True)
        if length is not None and len(traj) > length:
            traj.stamps = traj.stamps[:length]
            traj.poses = traj.poses[:length]
            gt_aligned.stamps = gt_aligned.stamps[:length]
            gt_aligned.poses = gt_aligned.poses[:length]
        ate = stats.ate(traj, gt_aligned).summarize(metric)
        r.update({"ATEt": ate.trans, "ATEr": ate.rot})

        for w in windows:
            rte = stats.rte(traj, gt_aligned, w).summarize(metric)
            r.update({f"RTEt_{w.name()}": rte.trans, f"RTEr_{w.name()}": rte.rot})

        results.append(r)

        if rr is not None and convert is not None and colors is not None and visualize:
            rr.log(
                traj.metadata.name,
                convert(traj, color=colors[index]),
                static=True,
            )

    return results


def _contains_dir(directory: Path) -> bool:
    return any(directory.is_dir() for directory in directory.glob("*"))


def evaluate(
    directories: list[Path],
    windows: list[stats.WindowKind],
    metric: stats.MetricKind,
    length: Optional[int] = None,
    visualize: bool = False,
) -> list[dict[str, Any]]:
    # Collect all bottom level directories
    bottom_level_dirs: list[Path] = []
    for directory in directories:
        for subdir in directory.glob("**/"):
            if not _contains_dir(subdir):
                bottom_level_dirs.append(subdir)

    # Compute them all in parallel
    results = Parallel(n_jobs=-2)(
        delayed(eval_dataset)(
            d,
            visualize,
            windows,
            metric,
            length,
        )
        for d in bottom_level_dirs
    )
    results = [r for r in results if r is not None]

    return list(itertools.chain.from_iterable(results))


@app.command("stats", no_args_is_help=True)
def evaluate_typer(
    directories: Annotated[
        list[Path], typer.Argument(help="Directory of results to evaluate.")
    ],
    visualize: Annotated[
        bool, typer.Option("--visualize", "-v", help="Visualize results.")
    ] = False,
    # output options
    sort: Annotated[
        Optional[str],
        typer.Option(
            "-s",
            "--sort",
            help="Sort results by the name of a column. Defaults to RTEt.",
            rich_help_panel="Output options",
        ),
    ] = None,
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
            "-f",
            "--filter",
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
    only_failed: Annotated[
        bool,
        typer.Option(
            "--only-failed",
            help="Only show results for trajectories that failed.",
            rich_help_panel="Filtering options",
        ),
    ] = False,
    hide_columns: Annotated[
        Optional[list[str]],
        typer.Option(
            "-s",
            "--hide-columns",
            help="Comma-separated list of columns to hide.",
            rich_help_panel="Output options",
        ),
    ] = None,
    print_columns: Annotated[
        bool,
        typer.Option(
            "--print-columns",
            help="Print the names of all available columns.",
            rich_help_panel="Output options",
        ),
    ] = False,
    # metric options
    w_meters: Annotated[
        Optional[list[float]],
        typer.Option(
            "--w-meters",
            help="Window size in meters for RTE computation. May be repeated. Defaults to 30m.",
            rich_help_panel="Metric options",
        ),
    ] = None,
    w_seconds: Annotated[
        Optional[list[float]],
        typer.Option(
            "--w-seconds",
            help="Window size in seconds for RTE computation. May be repeated. Defaults to none.",
            rich_help_panel="Metric options",
        ),
    ] = None,
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
) -> None:
    """
    Evaluate the results of experiments.
    """
    # ------------------------- Process all inputs ------------------------- #
    # Parse some of the options
    if only_complete and only_failed:
        raise typer.BadParameter(
            "Can only use one of --only-complete, --only-incomplete, or --only-failed."
        )

    # Parse the filtering options
    filter_method: Callable[[dict[str, Any]], bool]
    if filter_str is None:
        filter_method = lambda r: True  # noqa: E731
    else:
        filter_method = lambda r: eval(  # noqa: E731
            filter_str,
            {"__builtins__": None},
            {"np": np, **r},
        )

    original_filter = filter_method
    if only_complete:
        filter_method = lambda r: original_filter(r) and r["status"] == "complete"  # noqa: E731
    elif only_failed:
        filter_method = lambda r: original_filter(r) and r["status"] == "fail"  # noqa: E731

    windows: list[stats.WindowKind] = []
    if w_seconds is not None:
        windows.extend([stats.WindowSeconds(t) for t in w_seconds])
    if w_meters is not None:
        windows.extend([stats.WindowMeters(d) for d in w_meters])
    if len(windows) == 0:
        windows = [stats.WindowMeters(30.0)]

    if sort is None:
        sort = f"RTEt_{windows[0].name()}"

    c = Console()

    # ------------------------- Compute all results ------------------------- #
    results = evaluate(
        directories,
        windows,
        metric,
        length,
        visualize,
    )

    # ------------------------- Filter all results ------------------------- #
    try:
        results = [r for r in results if filter_method(r)]
    except Exception as e:
        print_warning(f"Error filtering results: {e}")

    # convert to polars dataframe for easier processing
    if len(results) == 0:
        print_warning("No results found.")
        return

    df = pl.DataFrame(results)

    # clean up timing
    df = df.with_columns(
        ((pl.col("sequence_length") / pl.col("total_elapsed")).alias("Hz"))
    )
    df = df.rename({"max_step_elapsed": "Max (s)"})

    # print columns if requested
    if print_columns:
        c.print("Available columns:")
        for col in df.columns:
            c.print(f" - {col}")
        return

    # hide some columns by default
    if hide_columns is None:
        hide_columns = []
    hide_columns.extend(["pipeline_version", "total_elapsed", "pipeline"])

    # delete unneeded columns
    remove_columns = [
        col
        for col in df.columns
        if col not in ["sequence", "name"]  # must keep these for later
        and not col.startswith("RTE")  # want to keep metrics as well
        and not col.startswith("ATE")
        and df[col].drop_nulls().n_unique() == 1  # remove if they're all the same
    ]
    remove_columns.extend([col for col in hide_columns if col in df.columns])
    df = df.drop(remove_columns)

    # sort if requested
    if sort not in df.columns:
        print_warning(f"Column {sort} not found, cannot sort.")
    else:
        df = df.sort(sort, descending=reverse)

    # ------------------------- Print ------------------------- #
    # Print sequence by sequence
    for sequence in df["sequence"].unique():
        df_sequence = df.filter(pl.col("sequence") == sequence)
        df_sequence = df_sequence.drop("sequence")
        if df_sequence.is_empty():
            continue

        table = Table(
            title=f"Results for {sequence}",
            box=box.ROUNDED,
            highlight=True,
            # show_lines=True,
            # header_style="bold magenta",
            # min_width =
        )

        for col in df_sequence.columns:
            table.add_column(
                col,
                justify="right" if df_sequence[col].dtype is pl.Float64 else "left",
                no_wrap=True,
            )

        for row in df_sequence.iter_rows():
            table.add_row(
                *[f"{x:.3f}" if isinstance(x, float) else str(x) for x in row]
            )

        c.print(table)
        c.print("\n")
