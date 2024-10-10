import atexit
import csv
from pathlib import Path

from evalio.types import SE3, Stamp

from .parser import DatasetBuilder, PipelineBuilder


class Writer:
    def __init__(self, path: Path, pipeline: PipelineBuilder, dataset: DatasetBuilder):
        path = path / dataset.dataset.name() / dataset.seq
        path.mkdir(parents=True, exist_ok=True)
        path /= f"{pipeline.name}.csv"

        # write metadata to the header
        self.path = path
        self.file = open(path, "w")
        self.file.write(f"# name: {pipeline.name}\n")
        self.file.write(f"# pipeline: {pipeline.pipeline.name()}\n")
        for key, value in pipeline.params.items():
            self.file.write(f"# {key}: {value}\n")
        self.file.write("#\n")
        self.file.write(f"# dataset: {dataset.dataset.name()}\n")
        self.file.write(f"# sequence: {dataset.seq}\n")
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
        # print(f"Wrote {self.index}")
        self.index += 1

    def close(self):
        self.file.close()


def save_gt(output: Path, dataset: DatasetBuilder):
    gt = dataset.build().ground_truth_corrected()
    path = output / dataset.dataset.name() / dataset.seq
    path.mkdir(parents=True, exist_ok=True)
    path = path / "gt.csv"
    with open(path, "w") as f:
        f.write(f"# dataset: {dataset.dataset.name()}\n")
        f.write(f"# sequence: {dataset.seq}\n")
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
