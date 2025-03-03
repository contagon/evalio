from ._cpp.types import *  # type: ignore  # noqa: F403
from ._cpp.types import SE3, Stamp  # type: ignore
from dataclasses import dataclass


@dataclass(kw_only=True)
class Trajectory:
    metadata: dict
    stamps: list[Stamp]
    poses: list[SE3]

    def __getitem__(self, idx: int) -> tuple[Stamp, SE3]:
        return self.stamps[idx], self.poses[idx]

    def __len__(self) -> int:
        return len(self.stamps)

    def __iter__(self):
        return iter(zip(self.stamps, self.poses))

    def transform_in_place(self, T: SE3):
        for i in range(len(self.poses)):
            self.poses[i] = self.poses[i] * T
