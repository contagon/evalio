import pytest
from evalio.cli.parser import DatasetBuilder
from evalio.datasets import Dataset

datasets = DatasetBuilder.all_datasets().values()


# Test to ensure all datasets implement the required attributes
@pytest.mark.parametrize("dataset", datasets)
def test_impl(dataset: type[Dataset]):
    attrs = [
        "data_iter",
        "ground_truth_raw",
        "url",
        "imu_T_lidar",
        "imu_T_gt",
        "imu_params",
        "lidar_params",
        "files",
    ]

    for a in attrs:
        assert getattr(dataset, a) != getattr(Dataset, a), (
            f"{dataset} should implement {a}"
        )
