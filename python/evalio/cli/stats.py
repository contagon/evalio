from pathlib import Path
from typing import Annotated, Any, Callable, Optional, cast

import polars as pl
import itertools

from evalio import types as ty, stats
from evalio.utils import print_warning
from rich.table import Table
from rich.console import Console
from rich import box

import distinctipy

from joblib import Parallel, delayed
from cyclopts import Group
from .types import Param


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
        r: dict[str, Any] = {}

        # compute hertz here before aligning
        hz = None
        if traj.metadata.total_elapsed is not None:
            hz = len(traj) / traj.metadata.total_elapsed

        if len(traj) > 0:
            # align to ground truth, copying ground truth by hand
            gt_aligned = ty.Trajectory(
                stamps=[ty.Stamp(s) for s in gt_og.stamps],
                poses=[ty.SE3(p) for p in gt_og.poses],
            )
            stats.align(traj, gt_aligned, in_place=True)

            # shrink to specified length
            if length is not None and len(traj) > length:
                traj.stamps = traj.stamps[:length]
                traj.poses = traj.poses[:length]
                gt_aligned.stamps = gt_aligned.stamps[:length]
                gt_aligned.poses = gt_aligned.poses[:length]

            # add metrics
            for w in windows:
                rte = stats.rte(traj, gt_aligned, w).summarize(metric)
                r.update({f"RTEt_{w.name()}": rte.trans, f"RTEr_{w.name()}": rte.rot})

            ate = stats.ate(traj, gt_aligned).summarize(metric)
            r.update({"ATEt": ate.trans, "ATEr": ate.rot})

        # add metadata
        r |= traj.metadata.to_dict()
        # add extra hz field
        r["hz"] = hz
        # flatten pipeline params
        r.update(r["pipeline_params"])
        del r["pipeline_params"]
        # remove type tag
        del r["type"]

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
    metric: stats.MetricKind = stats.MetricKind.sse,
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


og = Group("Output", sort_key=1)
fg = Group("Filtering", sort_key=2)
mg = Group("Metric", sort_key=3)


def evaluate_cli(
    directories: list[Path],
    /,
    *,
    visualize: Annotated[bool, Param("-v", og)] = False,
    # output options
    # sort: Annotated[Optional[str], sort_param] = None,
    # filtering options
    filter_str: Annotated[Optional[str], Param("-f", fg)] = None,
    only_complete: Annotated[bool, Param(group=fg)] = False,
    only_failed: Annotated[bool, Param(group=fg)] = False,
    # output options
    sort: Annotated[Optional[str], Param("-s", og)] = None,
    reverse: Annotated[bool, Param("-r", og)] = False,
    hide_columns: Annotated[Optional[list[str]], Param("-H", og)] = None,
    show_columns: Annotated[Optional[list[str]], Param("-S", og)] = None,
    print_columns: Annotated[bool, Param(group=og)] = False,
    # metric options
    w_meters: Annotated[Optional[list[float]], Param(group=mg)] = None,
    w_seconds: Annotated[Optional[list[float]], Param(group=mg)] = None,
    metric: Annotated[stats.MetricKind, Param("-m", mg)] = stats.MetricKind.sse,
    length: Annotated[Optional[int], Param("-l", mg)] = None,
) -> None:
    """Evaluate experiment results and display statistics.

    Args:
        directories (list[Path]): List of directories containing experiment results.
        visualize (bool, optional): Visualize resulting trajectories in rerun.
        sort (str, optional): Name of the column to sort results by. Defaults to RTEt for the first window.
        reverse (bool, optional): Reverse the sorting order.
        filter_str (str, optional): Python expression to filter result rows. Example: 'RTEt < 0.5'.
        only_complete (bool, optional): If True, only show results for completed trajectories.
        only_failed (bool, optional): If True, only show results for failed trajectories.
        hide_columns (list[str], optional): List of columns to hide from output.
        show_columns (list[str], optional): List of columns to force show in output.
        print_columns (bool, optional): Print the names of all available columns and exit.
        w_meters (list[float], optional): Add window size in meters for RTE computation. Defaults to [30.0].
        w_seconds (list[float], optional): Add window sizes in seconds for RTE computation.
        metric (stats.MetricKind, optional): Metric to use for ATE/RTE computation. Defaults to sse.
        length (int, optional): Specify subset of trajectory to evaluate.
    """
    # ------------------------- Process all inputs ------------------------- #
    # Parse some of the options
    if only_complete and only_failed:
        raise ValueError("Can only use one of --only-complete and --only-failed.")

    # Parse the filtering options
    filter_method: Callable[[dict[str, Any]], bool]
    if filter_str is None:
        filter_method = lambda r: True  # noqa: E731
    else:
        from asteval import Interpreter

        filter_method = lambda r: Interpreter(user_symbols=r).eval(filter_str)

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

    # rename length for brevity
    df = df.rename({"sequence_length": "len"})

    # print columns if requested
    if print_columns:
        c.print("Available columns:")
        for col in df.columns:
            c.print(f" - {col}")
        return

    # iterate through pipelines, finding unneeded columns
    unused_columns: set[str] = set()
    for pipeline in df["pipeline"].unique():
        df_pipeline = df.filter(pl.col("pipeline") == pipeline)
        unused_columns = unused_columns.union(
            col
            for col in df_pipeline.columns
            if df_pipeline[col].drop_nulls().n_unique() == 1
        )

    # add in a few more that we usually shouldn't need
    unused_columns.add("total_elapsed")
    unused_columns.add("pipeline")

    remove_columns = [
        col
        for col in unused_columns
        if col not in ["sequence", "name"]  # must keep these for later
        and not col.startswith("RTE")  # want to keep metrics as well
        and not col.startswith("ATE")
    ]

    # forcibly hide / show some columns
    if hide_columns is not None:
        for col in hide_columns:
            if col not in df.columns:
                print_warning(f"Column {col} not found, cannot hide.")
            else:
                remove_columns.append(col)

    if show_columns is not None:
        for col in show_columns:
            if col not in df.columns:
                print_warning(f"Column {col} not found, cannot show.")
            elif col in remove_columns:
                remove_columns.remove(col)

    df = df.drop(remove_columns)

    # rearrange for a more useful ordering (name to the left)
    cols = df.columns
    if "pipeline" in cols:
        cols.insert(0, cols.pop(cols.index("pipeline")))
    if "name" in cols:
        cols.insert(0, cols.pop(cols.index("name")))
    df = df.select(cols)

    # sort if requested
    if sort not in df.columns:
        print_warning(f"Column {sort} not found, cannot sort.")
    else:
        df = df.sort(sort, descending=reverse)

    # ------------------------- Print ------------------------- #
    # Print sequence by sequence
    for sequence in sorted(df["sequence"].unique()):
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
