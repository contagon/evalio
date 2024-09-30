import argparse
from pathlib import Path
import evalio
from tqdm import tqdm

from evalio._cpp.types import LidarMeasurement, ImuMeasurement  # type: ignore


def find_types(module, skip=None):
    found = {}
    found |= dict(
        (cls.name(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    found |= dict(
        (cls.nickname(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    return found


def eval():
    args = argparse.ArgumentParser()
    args.add_argument("-d", "--dataset", type=str, default="nc20/01")
    args.add_argument("-p", "--pipeline", type=str, default="kiss")
    args.add_argument("-o", "--output", type=Path, default="output")
    args.add_argument("-v", "--visualize", action="store_true")
    args = args.parse_args()

    # Load all options
    datasets = find_types(evalio.datasets, skip=evalio.datasets.Dataset)
    pipelines = find_types(evalio.pipelines, skip=evalio.pipelines.Pipeline)

    # Initialize dataset
    dataset_name, seq_name = args.dataset.split("/")
    Dataset = datasets[dataset_name]
    dataset = Dataset(seq_name)
    # TODO: Download check if not downloaded

    # Initialize pipeline
    Pipeline = pipelines[args.pipeline]
    pipe = Pipeline()
    pipe.set_imu_params(dataset.imu_params())
    pipe.set_lidar_params(dataset.lidar_params())
    pipe.set_imu_T_lidar(dataset.imu_T_lidar())
    # TODO: Other param setting
    # TODO: Very correct lidar point type
    pipe.initialize()

    if args.visualize:
        import rerun as rr

        rr.init("evalio", spawn=False)
        rr.connect("172.31.76.203:9876")

    # Run!
    # TODO: Saving???
    first_scan_done = False
    data_iter = iter(dataset)
    loop = tqdm(total=len(data_iter))
    for data in data_iter:
        if isinstance(data, ImuMeasurement):
            pipe.add_imu(data)
        elif isinstance(data, LidarMeasurement):
            pipe.add_lidar(data)
            pose = pipe.pose()

            if not first_scan_done and args.visualize:
                gt = dataset.ground_truth_corrected(pose)
                rr.log(
                    "gt", evalio.vis.poses_to_points(gt, color=[0, 0, 255]), static=True
                )
                rr.log(
                    "imu/lidar", evalio.vis.rerun(dataset.imu_T_lidar()), static=True
                )
                first_scan_done = True

            if args.visualize:
                rr.set_time_seconds("evalio_time", seconds=data.stamp.to_sec())
                rr.log("imu", evalio.vis.rerun(pose))
                rr.log("imu/lidar/frame", evalio.vis.rerun(data, use_intensity=True))
                # rr.log("map", evalio.vis.rerun(pipe.map(), color=[150, 150, 150]))

            loop.update()


if __name__ == "__main__":
    eval()
