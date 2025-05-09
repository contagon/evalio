import atexit
import csv
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


class TrajectoryWriter:
    def __init__(self, path: Path, pipeline: PipelineBuilder, dataset: DatasetBuilder):
        if path.suffix != ".csv":
            path = path / dataset.dataset.full_name
            path.mkdir(parents=True, exist_ok=True)
            path /= f"{pipeline.name}.csv"

        # write metadata to the header
        # TODO: Could probably automate this using pyserde somehow
        self.path = path
        self.file = open(path, "w")
        self.file.write(f"# name: {pipeline.name}\n")
        self.file.write(f"# pipeline: {pipeline.pipeline.name()}\n")
        self.file.write(f"# version: {pipeline.pipeline.version()}\n")
        for key, value in pipeline.params.items():
            self.file.write(f"# {key}: {value}\n")
        self.file.write("#\n")
        self.file.write(f"# dataset: {dataset.dataset.dataset_name()}\n")
        self.file.write(f"# sequence: {dataset.dataset.seq_name}\n")
        if dataset.length is not None:
            self.file.write(f"# length: {dataset.length}\n")
        self.file.write("#\n")
        self.file.write("# timestamp, x, y, z, qx, qy, qz, qw\n")

        self.writer = csv.writer(self.file)

        self.index = 0

        atexit.register(self.close)

    def write(self, stamp: Stamp, pose: SE3):
        self.writer.writerow(
            [
                f"{stamp.sec}.{stamp.nsec:09}",
                pose.trans[0],
                pose.trans[1],
                pose.trans[2],
                pose.rot.qx,
                pose.rot.qy,
                pose.rot.qz,
                pose.rot.qw,
            ]
        )
        # print(f"Wrote {self.index}")
        self.index += 1

    def close(self):
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
                    pose.trans[0],
                    pose.trans[1],
                    pose.trans[2],
                    pose.rot.qx,
                    pose.rot.qy,
                    pose.rot.qz,
                    pose.rot.qw,
                ]
            )
