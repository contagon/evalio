from pathlib import Path

from tqdm import tqdm

from evalio.types import ImuMeasurement, LidarMeasurement

from .parser import DatasetBuilder, PipelineBuilder


def run(
    pipelines: PipelineBuilder, datasets: DatasetBuilder, output: Path, visualize: bool
):
    if visualize:
        import rerun as rr
        import rerun.blueprint as rrb

        from evalio import vis as evis

        rr.connect("0.0.0.0:9876")

    for dbuilder in datasets:
        for pbuilder in pipelines:
            # Setup all the things
            print(f"Running {pbuilder} on {dbuilder}")
            dataset = dbuilder.build()
            pipe = pbuilder.build(dataset)

            if visualize:
                rr.init(
                    "evalio",
                    spawn=False,
                    default_blueprint=rrb.Spatial3DView(
                        overrides={"imu/lidar": [rrb.components.Visible(False)]}
                    ),
                )

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

                    if not first_scan_done and visualize:
                        gt = dataset.ground_truth_corrected(pose)
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
                        # rr.log("map", evis.rerun(pipe.map(), color=[150, 150, 150]))

                    loop.update()
                    if loop.n >= length:
                        loop.close()
                        break
