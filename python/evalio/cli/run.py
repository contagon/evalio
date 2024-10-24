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
                    features = pipe.add_lidar(data)
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

            writer.close()

    eval([output], False, "atet")
