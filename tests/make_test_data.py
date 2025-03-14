from evalio.cli.parser import DatasetBuilder
from pathlib import Path
import pickle  # noqa: F401

dataset_classes = DatasetBuilder._all_datasets()
datasets = [
    cls(cls.sequences()[0])
    for cls in dataset_classes.values()
    if cls.check_download(cls.sequences()[0])
]


data_dir = Path("tests/data")
data_dir.mkdir(parents=True, exist_ok=True)

# Make actual data
for d in datasets:
    # load lidar
    lidar = d.get_one_lidar()
    imu = d.get_one_imu()
    # cache them
    pickle.dump(imu, open(data_dir / f"imu_{d.name()}.pkl", "wb"))
    pickle.dump(lidar, open(data_dir / f"lidar_{d.name()}.pkl", "wb"))
