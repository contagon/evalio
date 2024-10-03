import argparse
from pathlib import Path

from .download import download_datasets


def main():
    args = argparse.ArgumentParser(
        "Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets"
    )
    subparsers = args.add_subparsers(dest="command")

    # downloading
    download = subparsers.add_parser("download", help="Download datasets")
    download.add_argument("dataset", type=str, help="Dataset(s) to download", nargs="+")

    # eval
    eval = subparsers.add_parser("run", help="Run a pipeline on a specific dataset")
    eval.add_argument("-d", "--dataset", type=str)
    eval.add_argument("-p", "--pipeline", type=str)
    eval.add_argument("-o", "--output", type=Path)

    # parse
    args = args.parse_args()
    if args.command == "download":
        download_datasets(args.dataset)
    elif args.command == "run":
        print("Not implemented yet")


if __name__ == "__main__":
    main()
