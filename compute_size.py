from typing import Any
from evalio import datasets as ds, types as ty
import numpy as np

# ------------------------- Compute Total Dataset Time Length ------------------------- #
all_sequences = ds.all_sequences()
all_sequences_with_len = [
    s for s in all_sequences.values() if s.quick_len() is not None
]

temp: list[float] = list(
    s.quick_len() / s.lidar_params().rate  # type: ignore
    for s in all_sequences.values()
    if s.quick_len() is not None
)
total_length = sum(temp)  # type: ignore
print(total_length / 60 / 60, "hours")

# ------------------------- Compute Total Dataset Distance Length ------------------------- #


def compute_length(traj: ty.Trajectory[Any]):
    dist = np.zeros(len(traj))
    for i in range(1, len(traj)):
        dist[i] = ty.SE3.distance(traj.poses[i], traj.poses[i - 1])

    return np.sum(dist)


print(compute_length(ds.Hilti2022.basement_2.ground_truth()))
temp = list(
    compute_length(s.ground_truth())
    for s in all_sequences.values()
    if s.is_downloaded()
)
total_length = sum(temp)  # type: ignore
print(total_length / 1000, "km")

not_downloaded = [s for s in all_sequences.values() if not s.is_downloaded()]
print("Not downloaded:")
for s in not_downloaded:
    print(s.name)
