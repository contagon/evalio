import pickle  # noqa: F401
from pathlib import Path

import evalio.datasets as ds

dataset_classes = ds.all_datasets()
datasets = [
    cls(cls.sequences()[0])
    for cls in dataset_classes.values()
    if cls.is_downloaded(cls.sequences()[0])
]


data_dir = Path("tests/data")
data_dir.mkdir(parents=True, exist_ok=True)

# Make actual data
for d in datasets:
    imu_file = data_dir / f"imu_{d.dataset_name()}.pkl"
    if not imu_file.exists():
        print(f"Extracting IMU data for {d.dataset_name()}...")
        imu = d.get_one_imu()
        with open(imu_file, "wb") as f:
            pickle.dump(imu, f)

    lidar_file = data_dir / f"lidar_{d.dataset_name()}.pkl"
    if not lidar_file.exists():
        print(f"Extracting LiDAR data for {d.dataset_name()}...")
        lidar = d.get_one_lidar()
        with open(lidar_file, "wb") as f:
            pickle.dump(lidar, f)
