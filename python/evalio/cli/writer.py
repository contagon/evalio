import atexit
import csv
from enum import Enum, auto
from pathlib import Path
from typing import Sequence
import yaml
from evalio.types import SE3, Stamp

from .parser import DatasetBuilder, PipelineBuilder


def save_config(
    pipelines: Sequence[PipelineBuilder],
    datasets: Sequence[DatasetBuilder],
    output: Path,
):
    # If it's just a file, don't save the entire config file
    if output.suffix == ".csv":
        return

    print(f"Saving config to {output}")

    output.mkdir(parents=True, exist_ok=True)
    path = output / "config.yaml"

    out = dict()
    out["datasets"] = [d.as_dict() for d in datasets]
    out["pipelines"] = [p.as_dict() for p in pipelines]

    with open(path, "w") as f:
        yaml.dump(out, f)


class ExperimentStatus(Enum):
    FINISHED = auto()
    STARTED = auto()
    FAILED = auto()
    NOT_STARTED = auto()


class TrajectoryWriter:
    def __init__(self, path: Path, pipeline: PipelineBuilder, dataset: DatasetBuilder):
        if path.suffix != ".csv":
            path = path / dataset.dataset.full_name
            path.mkdir(parents=True, exist_ok=True)
            path /= f"{pipeline.name}.csv"

        self.path = path
        atexit.register(self.close)

        # TODO: Could probably automate this using pyserde somehow
        params = "\n".join(
            f"# {key}: {value}" for key, value in pipeline.params.items()
        )
        self.metadata = f"""
# name: {pipeline.name}
# pipeline: {pipeline.pipeline.name()}
# version: {pipeline.pipeline.version()}
{params}
# dataset: {dataset.dataset.dataset_name()}
# sequence: {dataset.dataset.seq_name}
# length: {dataset.length if dataset.length is not None else "unknown"}
#
# timestamp, x, y, z, qx, qy, qz, qw
"""

    def status(self) -> ExperimentStatus:
        if not self.path.exists():
            return ExperimentStatus.NOT_STARTED

        with self.path.open("r") as f:
            lines = f.readlines()
            if not lines:
                return ExperimentStatus.STARTED

            last_line = lines[-1].strip()
            if last_line == "# status: complete":
                return ExperimentStatus.FINISHED
            elif last_line == "# status: fail":
                return ExperimentStatus.FAILED
            else:
                return ExperimentStatus.STARTED

    def start(self):
        # write metadata to the header
        self.file = open(self.path, "w")
        self.file.write(self.metadata)
        self.writer = csv.writer(self.file)
        self.index = 0

    def write(self, stamp: Stamp, pose: SE3):
        self.writer.writerow(
            [
                f"{stamp.sec}.{stamp.nsec:09}",
                pose.trans[0],  # type: ignore
                pose.trans[1],  # type: ignore
                pose.trans[2],  # type: ignore
                pose.rot.qx,
                pose.rot.qy,
                pose.rot.qz,
                pose.rot.qw,
            ]
        )
        self.index += 1

    def finish(self):
        with open(self.path, "a") as f:
            f.write("# status: complete")

    def fail(self):
        with open(self.path, "a") as f:
            f.write("# status: fail")

    def close(self):
        if hasattr(self, "file"):
            self.file.close()


def save_gt(output: Path, dataset: DatasetBuilder):
    if output.suffix == ".csv":
        return

    gt = dataset.build().ground_truth()
    path = output / dataset.dataset.full_name
    path.mkdir(parents=True, exist_ok=True)
    path = path / "gt.csv"
    with open(path, "w") as f:
        f.write(f"# dataset: {dataset.dataset.dataset_name()}\n")
        f.write(f"# sequence: {dataset.dataset.seq_name}\n")
        f.write("# gt: True\n")
        f.write("#\n")
        f.write("# timestamp, x, y, z, qx, qy, qz, qw\n")
        writer = csv.writer(f)
        for stamp, pose in gt:
            writer.writerow(
                [
                    stamp.to_sec(),
                    pose.trans[0],  # type: ignore
                    pose.trans[1],  # type: ignore
                    pose.trans[2],  # type: ignore
                    pose.rot.qx,
                    pose.rot.qy,
                    pose.rot.qz,
                    pose.rot.qw,
                ]
            )
