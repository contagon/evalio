from pathlib import Path

from tqdm import tqdm

from evalio.types import ImuMeasurement, LidarMeasurement

from .parser import DatasetBuilder, PipelineBuilder
from .writer import Writer, save_gt
from uuid import uuid4


def run(
    pipelines: list[PipelineBuilder],
    datasets: list[DatasetBuilder],
    output: Path,
    visualize: bool,
):
    if visualize:
        import rerun as rr
        import rerun.blueprint as rrb

        from evalio import vis as evis

    print(f"Running {len(pipelines)} pipelines on {len(datasets)} datasets\n")

    for dbuilder in datasets:
        save_gt(output, dbuilder)

        for pbuilder in pipelines:
            print(f"Running {pbuilder} on {dbuilder}")
            # Build everything
            dataset = dbuilder.build()
            pipe = pbuilder.build(dataset)
            writer = Writer(output, pbuilder, dbuilder)

            # Initialize params
            first_scan_done = False
            data_iter = iter(dataset)
            length = len(data_iter)
            if dbuilder.length is not None and dbuilder.length < length:
                length = dbuilder.length
            loop = tqdm(total=length)

            # Run the pipeline
            for data in data_iter:
                if isinstance(data, ImuMeasurement):
                    pipe.add_imu(data)
                elif isinstance(data, LidarMeasurement):
                    pipe.add_lidar(data)
                    pose = pipe.pose()
                    writer.write(data.stamp, pose)

                    if not first_scan_done and visualize:
                        gt = dataset.ground_truth_corrected(pose)
                        gt = [pose for _, pose in gt]
                        rr.new_recording(
                            str(dbuilder),
                            make_default=True,
                            recording_id=uuid4(),
                        )
                        rr.connect(
                            "0.0.0.0:9876",
                            default_blueprint=rrb.Spatial3DView(
                                overrides={"imu/lidar": [rrb.components.Visible(False)]}
                            ),
                        )
                        rr.log(
                            "gt",
                            evis.poses_to_points(gt, color=[0, 0, 255]),
                            static=True,
                        )
                        rr.log(
                            "imu/lidar",
                            evis.rerun(dataset.imu_T_lidar()),
                            static=True,
                        )
                    first_scan_done = True

                    if visualize:
                        rr.set_time_seconds("evalio_time", seconds=data.stamp.to_sec())
                        rr.log("imu", evis.rerun(pose))
                        # rr.log("imu/lidar/frame", evis.rerun(data, use_intensity=True))

                    loop.update()
                    if loop.n >= length:
                        loop.close()
                        break

            writer.close()
