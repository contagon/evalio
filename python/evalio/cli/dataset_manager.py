from pathlib import Path
from evalio.utils import print_warning
from .parser import DatasetBuilder
import typer
from typing import Annotated, cast
from .completions import DatasetArg

from evalio.datasets import RosbagIter

from rosbags.rosbag1 import (
    Reader as Reader1,
    Writer as Writer1,
)
from rosbags.rosbag2 import (
    Reader as Reader2,
    StoragePlugin,
    Writer as Writer2,
)
from rosbags.typesys import get_typestore, Stores
from rosbags.interfaces import ConnectionExtRosbag2

app = typer.Typer()


@app.command(no_args_is_help=True)
def dl(datasets: DatasetArg) -> None:
    """
    Download datasets
    """
    # parse all datasets
    valid_datasets = DatasetBuilder.parse(datasets)

    # Check if already downloaded
    to_download = []
    for builder in valid_datasets:
        if builder.is_downloaded():
            print(f"Skipping download for {builder}, already exists")
        else:
            to_download.append(builder)

    if len(to_download) == 0:
        print("Nothing to download, finishing")
        return

    # download each dataset
    print("Will download: ")
    for builder in to_download:
        print(f"  {builder}")
    print()

    for builder in to_download:
        print(f"---------- Beginning {builder} ----------")
        try:
            builder.download()
        except Exception as e:
            print(f"Error downloading {builder}\n: {e}")
        print(f"---------- Finished {builder} ----------")


@app.command(no_args_is_help=True)
def rm(
    datasets: DatasetArg,
    force: Annotated[
        bool,
        typer.Option(
            "--force",
            "-f",
            prompt="Are you sure you want to delete these datasets?",
            help="Force deletion without confirmation",
        ),
    ] = False,
):
    """
    Remove dataset(s)

    If --force is not used, will ask for confirmation.
    """
    # parse all datasets
    to_remove = DatasetBuilder.parse(datasets)

    print("Will remove: ")
    for builder in to_remove:
        print(f"  {builder}")
    print()

    for builder in to_remove:
        print(f"---------- Beginning {builder} ----------")
        try:
            print(f"Removing from {builder.dataset.folder}")
            for f in builder.dataset.files():
                print(f"  Removing {f}")
                (builder.dataset.folder / f).unlink()
        except Exception as e:
            print(f"Error removing {builder}\n: {e}")
        print(f"---------- Finished {builder} ----------")


def filter_ros1(bag: Path, topics: list[str]) -> None:
    print(bag)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    bag_temp = bag.with_suffix(".temp.bag")

    with Reader1(bag) as reader, Writer1(bag_temp) as writer:
        # Gather all the connections (messages) that we want to keep
        conn_write = {}
        conn_read = []
        other_topics = False
        for conn in reader.connections:
            if conn.topic not in topics:
                other_topics = True
                continue

            conn_write[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                typestore=typestore,
            )
            conn_read.append(conn)

        if not other_topics:
            print("-- Skipping, no other topics found, filtering not needed")
            return

        # Save messages
        print("-- Creating temporary bag...")
        for conn, timestamp, data in reader.messages(connections=conn_read):
            writer.write(conn_write[conn.id], timestamp, data)

    # Replace the original bag with the filtered one
    print("-- Replacing original with temporary...")
    bag_temp.replace(bag)


def filter_ros2(bag: Path, topics: list[str]) -> None:
    print(bag)
    typestore = get_typestore(Stores.ROS2_FOXY)
    bag_temp = bag.parent / (bag.name + "_temp")

    if len(list(bag.glob("*.mcap"))) > 0:
        storage = StoragePlugin.MCAP
    elif len(list(bag.glob("*.db3"))) > 0:
        storage = StoragePlugin.SQLITE3
    else:
        print_warning("No valid storage format found, cannot filter ros2 bag")
        return

    with Reader2(bag) as reader, Writer2(bag_temp, storage_plugin=storage) as writer:
        # Gather all the connections (messages) that we want to keep
        conn_write = {}
        conn_read = []
        other_topics = False
        for conn in reader.connections:
            if conn.topic not in topics:
                other_topics = True
                continue

            ext = cast("ConnectionExtRosbag2", conn.ext)
            conn_write[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                typestore=typestore,
                serialization_format=ext.serialization_format,
                offered_qos_profiles=ext.offered_qos_profiles,
            )
            conn_read.append(conn)

        if not other_topics:
            print("-- Skipping, no other topics found, filtering not needed")
            return

        # Save messages
        print("-- Creating temporary bag...")
        for conn, timestamp, data in reader.messages(connections=conn_read):
            writer.write(conn_write[conn.id], timestamp, data)

    # Replace the original bag with the filtered one
    print("-- Replacing original with temporary...")
    for f in bag_temp.glob("*"):
        # replace file with file of same extension
        to_replace = [p for p in bag.glob("*") if p.suffix == f.suffix]
        if len(to_replace) != 1:
            print_warning("Something went wrong, unclear which file to replace")
            continue
        f.replace(to_replace[0])


@app.command(no_args_is_help=True)
def filter(
    datasets: DatasetArg,
    force: Annotated[
        bool,
        typer.Option(
            "--force",
            "-f",
            prompt="Are you sure you want to filter these datasets?",
            help="Force deletion without confirmation",
        ),
    ] = False,
):
    """
    Filter rosbag dataset(s) to only include lidar and imu data. Useful for shrinking disk size.
    """
    # parse all datasets
    valid_datasets = DatasetBuilder.parse(datasets)

    # Check if already downloaded
    to_filter = []
    for builder in valid_datasets:
        if not builder.is_downloaded():
            print(f"Skipping filter for {builder}, not downloaded")
        else:
            to_filter.append(builder)

    print("Will filter: ")
    for builder in to_filter:
        print(f"  {builder}")
    print()

    for builder in to_filter:
        print(f"---------- Filtering {builder} ----------")
        # try:
        data = builder.build().data_iter()
        if not isinstance(data, RosbagIter):
            print(f"{builder} is not a RosbagDataset, skipping filtering")
            continue

        is2 = (data.path[0] / "metadata.yaml").exists()
        topics = [data.imu_topic, data.lidar_topic]

        # Find all bags to filter
        if is2:
            bags = data.path
        else:
            bags = []
            for path in data.path:
                if path.is_dir():
                    bags += list(path.glob("*.bag"))
                elif path.suffix == ".bag":
                    bags.append(path)

        if len(bags) == 0:
            print_warning("Something went wrong, no bags found")

        print(f"Found {len(bags)} {'ros2' if is2 else 'ros1'} bags to filter")

        # Filtering each bag
        for bag in bags:
            if is2:
                filter_ros2(bag, topics)
            else:
                filter_ros1(bag, topics)

        # except Exception as e:
        #     print(f"Error filtering {builder}\n: {e}")
        print(f"---------- Finished {builder} ----------")
