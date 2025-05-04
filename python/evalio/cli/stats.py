from copy import deepcopy
from enum import StrEnum, auto
from pathlib import Path
from typing import Annotated, Optional, Sequence

import numpy as np
from dataclasses import dataclass
from rich.table import Table
from rich.console import Console
from rich import box

from evalio.types import Stamp, Trajectory, SE3

import typer

from typing import cast


app = typer.Typer()


# ------------------------- Methods for computing metrics ------------------------- #
class MetricKind(StrEnum):
    mean = auto()
    median = auto()
    sse = auto()


@dataclass(kw_only=True)
class Metric:
    trans: float
    rot: float


@dataclass(kw_only=True)
class Error:
    # Shape: (n,)
    trans: np.ndarray
    rot: np.ndarray

    def summarize(self, metric: MetricKind) -> Metric:
        match metric:
            case MetricKind.mean:
                return self.mean()
            case MetricKind.median:
                return self.median()
            case MetricKind.sse:
                return self.sse()

    def mean(self) -> Metric:
        return Metric(rot=self.rot.mean(), trans=self.trans.mean())

    def sse(self) -> Metric:
        length = len(self.rot)
        return Metric(
            rot=float(np.sqrt(self.rot @ self.rot / length)),
            trans=float(np.sqrt(self.trans @ self.trans / length)),
        )

    def median(self) -> Metric:
        return Metric(
            rot=cast(float, np.median(self.rot)),
            trans=cast(float, np.median(self.trans)),
        )


class ExperimentResults:
    metadata: dict
    stamps: list[Stamp]
    poses: list[SE3]
    gts: list[SE3]

    def __init__(
        self, gt_og: Trajectory, traj: Trajectory, length: Optional[int] = None
    ):
        self.metadata = traj.metadata

        gt = deepcopy(gt_og)
        Trajectory.align(traj, gt, in_place=True)

        self.stamps = traj.stamps
        self.poses = traj.poses
        self.gts = gt.poses

        if length is not None and length < len(self.stamps):
            self.stamps = self.stamps[:length]
            self.poses = self.poses[:length]
            self.gts = self.gts[:length]

    def __len__(self) -> int:
        return len(self.stamps)

    @staticmethod
    def _compute_metric(gts: list[SE3], poses: list[SE3]) -> Error:
        assert len(gts) == len(poses)

        error_t = np.zeros(len(gts))
        error_r = np.zeros(len(gts))
        for i, (gt, pose) in enumerate(zip(gts, poses)):
            delta = gt.inverse() * pose
            error_t[i] = np.sqrt(delta.trans @ delta.trans)  # type: ignore
            r_diff = delta.rot.log()
            error_r[i] = np.sqrt(r_diff @ r_diff) * 180 / np.pi  # type: ignore

        return Error(rot=error_r, trans=error_t)

    def ate(self) -> Error:
        return self._compute_metric(self.gts, self.poses)

    def rte(self, window: int = 100) -> Error:
        if window <= 0:
            raise ValueError("Window size must be positive")

        window_deltas_poses = []
        window_deltas_gts = []
        for i in range(len(self.gts) - window):
            window_deltas_poses.append(self.poses[i].inverse() * self.poses[i + window])
            window_deltas_gts.append(self.gts[i].inverse() * self.gts[i + window])

        return self._compute_metric(window_deltas_gts, window_deltas_poses)


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
            exp = ExperimentResults(gt_og, traj, length)
            ate = exp.ate().summarize(metric)
            rte = exp.rte(window_size).summarize(metric)
            r = {
                "name": traj.metadata["name"],
                "RTEt": rte.trans,
                "RTEr": rte.rot,
                "ATEt": ate.trans,
                "ATEr": ate.rot,
                "length": len(exp),
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
