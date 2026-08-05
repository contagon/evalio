from typing import Any

import numpy as np
from evalio import datasets as ds
from evalio import types as ty

# ------------------------- Compute Total Dataset Time Length ------------------------- #
all_sequences = ds.all_sequences()
all_sequences_with_len = [
    s for s in all_sequences.values() if s.quick_len() is not None
]

temp: list[float] = [
    s.quick_len() / s.lidar_params().rate  # type: ignore
    for s in all_sequences.values()
    if s.quick_len() is not None
]
total_length = sum(temp)  # type: ignore
print(total_length / 60 / 60, "hours")

# ------------------------- Compute Total Dataset Distance Length ------------------------- #


def compute_length(traj: ty.Trajectory[Any]):
    dist = np.zeros(len(traj))
    for i in range(1, len(traj)):
        dist[i] = ty.SE3.distance(traj.poses[i], traj.poses[i - 1])

    return np.sum(dist)


# pyrefly: ignore [bad-argument-type]
print(compute_length(ds.Hilti2022.basement_2.ground_truth()))
temp = [
    # pyrefly: ignore [bad-argument-type]
    compute_length(s.ground_truth())
    for s in all_sequences.values()
    if s.is_downloaded()
]
total_length = sum(temp)  # type: ignore
print(total_length / 1000, "km")

not_downloaded = [s for s in all_sequences.values() if not s.is_downloaded()]
print("Not downloaded:")
for s in not_downloaded:
    print(s.name)
