from pathlib import Path
from tqdm import tqdm

from evalio.types import ImuMeasurement, LidarMeasurement
from evalio.rerun import RerunVis

from .parser import DatasetBuilder, PipelineBuilder
from .writer import TrajectoryWriter, save_config, save_gt
from .stats import eval


def plural(num: int, word: str) -> str:
    return f"{num} {word}{ 's' if num > 1 else ''}"


def run(
    pipelines: list[PipelineBuilder],
    datasets: list[DatasetBuilder],
    output: Path,
    vis: RerunVis,
):
    print(
        f"Running {plural(len(pipelines), 'pipeline')} on {plural(len(datasets), 'dataset')} => {plural(len(pipelines) * len(datasets), 'experiment')}"
    )
    if all(d.length is not None for d in datasets):
        dtime = sum(d.length / d.dataset.lidar_params().rate for d in datasets)  # type: ignore
        dtime *= len(pipelines)
        if dtime > 3600:
            print(f"Total estimated: {dtime / 3600:.2f} hours")
        elif dtime > 60:
            print(f"Total estimated: {dtime / 60:.2f} minutes")
        else:
            print(f"Total estimated: {dtime:.2f} seconds")
    else:
        print("Total time is unknown, missing dataset information")
    print(f"Output will be saved to {output}\n")

    save_config(pipelines, datasets, output)

    for dbuilder in datasets:
        save_gt(output, dbuilder)

        for pbuilder in pipelines:
            print(f"Running {pbuilder} on {dbuilder}")
            # Build everything
            dataset = dbuilder.build()
            pipe = pbuilder.build(dataset)
            writer = TrajectoryWriter(output, pbuilder, dbuilder)

            gt_list = dataset.ground_truth()  # already in the IMU frame!
            stamps = [s.to_nsec() for s in gt_list.stamps]
            gt = dict(zip(stamps, gt_list.poses))
            imu_T_lidar = dataset.imu_T_lidar()

            # Initialize params
            first_scan_done = False
            data_iter = iter(dataset)
            length = len(data_iter)
            if dbuilder.length is not None and dbuilder.length < length:
                length = dbuilder.length
            loop = tqdm(total=length)

            # Run the pipeline
            # try:
            last_gt = None
            for data in data_iter:
                if isinstance(data, ImuMeasurement):
                    pipe.add_imu(data)
                elif isinstance(data, LidarMeasurement):
                    # TODO: Probably need to actually align the timestamps... for enwide timestamps don't line up
                    if data.stamp.to_nsec() not in gt:
                        continue
                    this_gt = gt[data.stamp.to_nsec()] * imu_T_lidar
                    if last_gt is None:
                        init = None
                    else:
                        init = last_gt.inverse() * this_gt
                    last_gt = this_gt

                    features = pipe.add_lidar(data, init)
                    pose = pipe.pose()
                    writer.write(data.stamp, pose)

                    if not first_scan_done:
                        vis.new_recording(dataset)
                        first_scan_done = True

                    vis.log(data, features, pose)

                    loop.update()
                    if loop.n >= length:
                        loop.close()
                        break

            # except Exception as e:
            #     print("Failed to run", e)
            #     loop.close()

            writer.close()

    eval([output], False, "atet")
