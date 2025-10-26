import shutil
from pathlib import Path
from typing import Annotated, cast

import typer
from rosbags.interfaces import Connection, ConnectionExtRosbag2
from rosbags.rosbag1 import (
    Reader as Reader1,
)
from rosbags.rosbag1 import (
    Writer as Writer1,
)
from rosbags.rosbag2 import (
    Reader as Reader2,
)
from rosbags.rosbag2 import (
    StoragePlugin,
)
from rosbags.rosbag2 import (
    Writer as Writer2,
)
from rosbags.typesys import Stores, get_typestore

import evalio.datasets as ds
from evalio.utils import print_warning

from .completions import DatasetArg

app = typer.Typer()


def parse_datasets(
    datasets: DatasetArg,
) -> list[ds.Dataset]:
    """
    Parse datasets from command line argument
    """
    # parse all datasets
    valid_datasets = ds.parse_config(datasets)
    if isinstance(valid_datasets, ds.DatasetConfigError):
        print_warning(f"Error parsing datasets: {valid_datasets}")
        return []
    return [b[0] for b in valid_datasets]


@app.command(no_args_is_help=True)
def dl(datasets: DatasetArg) -> None:
    """
    Download datasets
    """
    # parse all datasets
    valid_datasets = parse_datasets(datasets)

    # Check if already downloaded
    to_download: list[ds.Dataset] = []
    for dataset in valid_datasets:
        if dataset.is_downloaded():
            print(f"Skipping download for {dataset}, already exists")
        else:
            to_download.append(dataset)

    if len(to_download) == 0:
        print("Nothing to download, finishing")
        return

    # download each dataset
    print("Will download:")
    for dataset in to_download:
        print(f"  {dataset}")
    print()

    for dataset in to_download:
        print(f"---------- Beginning {dataset} ----------")
        try:
            dataset.download()
        except Exception as e:
            print(f"Error downloading {dataset}\n: {e}")
        print(f"---------- Finished {dataset} ----------")


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
    to_remove = parse_datasets(datasets)

    print("Will remove:")
    for dataset in to_remove:
        print(f"  {dataset}")
    print()

    for dataset in to_remove:
        print(f"---------- Beginning {dataset} ----------")
        try:
            print(f"Removing from {dataset.folder}")
            for f in dataset.files():
                print(f"  Removing {f}")
                if (dataset.folder / f).is_file():
                    (dataset.folder / f).unlink()
                else:
                    shutil.rmtree(dataset.folder / f, ignore_errors=True)
        except Exception as e:
            print(f"Error removing {dataset}\n: {e}")
        print(f"---------- Finished {dataset} ----------")


def filter_ros1(bag: Path, topics: list[str]) -> None:
    print(bag)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    bag_temp = bag.with_suffix(".temp.bag")

    with Reader1(bag) as reader, Writer1(bag_temp) as writer:
        # Gather all the connections (messages) that we want to keep
        conn_write: dict[int, Connection] = {}
        conn_read: list[Connection] = []
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

    if len(mcap := list(bag.glob("*.mcap"))) == 1:
        storage = StoragePlugin.MCAP
        storage_file = mcap[0]
    elif len(sqlite3 := list(bag.glob("*.db3"))) == 1:
        storage = StoragePlugin.SQLITE3
        storage_file = sqlite3[0]
    else:
        print_warning("No valid storage format found, cannot filter ros2 bag")
        return

    bag_temp = storage_file.parent.parent / storage_file.stem

    assert not bag_temp.exists(), (
        f"Temporary bag {bag_temp} already exists, please remove it first"
    )

    with Reader2(bag) as reader, Writer2(bag_temp, storage_plugin=storage) as writer:
        # Gather all the connections (messages) that we want to keep
        conn_write: dict[int, Connection] = {}
        conn_read: list[Connection] = []
        other_topics = False
        for conn in reader.connections:
            if conn.topic not in topics:
                other_topics = True
                continue

            ext = cast(ConnectionExtRosbag2, conn.ext)
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
    (bag_temp / (bag_temp.stem + storage_file.suffix)).replace(storage_file)
    (bag_temp / "metadata.yaml").replace(bag / "metadata.yaml")
    bag_temp.rmdir()


@app.command(no_args_is_help=True)
def filter(
    datasets: DatasetArg,
    force: Annotated[
        bool,
        typer.Option(
            "--force",
            "-f",
            prompt="Are you sure you want to filter these datasets? This is slightly experimental, please make sure the data has a copy somewhere!",
            help="Force deletion without confirmation",
        ),
    ] = False,
):
    """
    Filter rosbag dataset(s) to only include lidar and imu data. Useful for shrinking disk size.
    """
    # parse all datasets
    valid_datasets = parse_datasets(datasets)

    # Check if already downloaded
    to_filter: list[ds.Dataset] = []
    for dataset in valid_datasets:
        if not dataset.is_downloaded():
            print(f"Skipping filter for {dataset}, not downloaded")
        else:
            to_filter.append(dataset)

    print("Will filter: ")
    for dataset in to_filter:
        print(f"  {dataset}")
    print()

    for dataset in to_filter:
        print(f"---------- Filtering {dataset} ----------")
        # try:
        data = dataset.data_iter()
        if not isinstance(data, ds.RosbagIter):
            print(f"{dataset} is not a RosbagDataset, skipping filtering")
            continue

        is2 = (data.path[0] / "metadata.yaml").exists()
        topics = [data.imu_topic, data.lidar_topic]

        # Find all bags to filter
        if is2:
            bags = data.path
        else:
            bags: list[Path] = []
            for path in data.path:
                if path.is_dir():
                    bags += list(path.glob("*.bag"))
                elif path.suffix == ".bag":
                    bags.append(path)

        if len(bags) == 0:
            print_warning("Something went wrong, no bags found")
            continue

        print(f"Found {len(bags)} {'ros2' if is2 else 'ros1'} bags to filter")

        # Filtering each bag
        for bag in bags:
            if is2:
                filter_ros2(bag, topics)
            else:
                filter_ros1(bag, topics)

        # except Exception as e:
        #     print(f"Error filtering {dataset}\n: {e}")
        print(f"---------- Finished {dataset} ----------")
