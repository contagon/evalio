from evalio.cli.parser import DatasetBuilder
from pathlib import Path
import pickle  # noqa: F401
import time

dataset_classes = DatasetBuilder._all_datasets()
datasets = [cls(cls.sequences()[0]) for cls in dataset_classes.values()]


data_dir = Path("tests/data")
data_dir.mkdir(parents=True, exist_ok=True)


# Make actual data
for d in datasets:
    # load lidar
    lidar = d.first_n_lidar_scans(10)
    assert len(lidar) == 10, "Lidar scans not found"

    # load imu
    imu = d.first_n_imu_measurement(100)
    assert len(imu) == 100, "IMU measurements not found"

    # cache them
    pickle.dump(imu, open(data_dir / f"imu_{d.name()}.pkl", "wb"))
    pickle.dump(lidar, open(data_dir / f"lidar_{d.name()}.pkl", "wb"))


# # Make timing data
# for d in datasets:
#     print(f"{d.name()}: ", end="", flush=True)
#     # do it once to make it fresh
#     start = time.time()
#     iterator = d.lidar_iter()
#     for _ in iterator:
#         pass
#     end = time.time()
#     print(f"{end - start:.2f}s, ", end="", flush=True)

#     # time the second one
#     start = time.time()
#     iterator = d.lidar_iter()
#     for _ in iterator:
#         pass
#     end = time.time()

#     print(f"{end - start:.2f}s", flush=True)
#     with open(data_dir / f"lidar_time_{d.name()}.txt", "w") as f:
#         f.write(f"{end - start:.4f}")
