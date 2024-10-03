import argparse
from pathlib import Path

from .download import download_datasets
from .parser import parse_config
from .ls import ls


def main():
    args = argparse.ArgumentParser(
        "Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets"
    )
    subparsers = args.add_subparsers(dest="command")

    # downloading
    download = subparsers.add_parser("download", help="Download datasets")
    download.add_argument("dataset", type=str, help="Dataset(s) to download", nargs="+")

    # ls
    ls_opt = subparsers.add_parser("ls", help="List available datasets and pipelines")
    ls_opt.add_argument("options", type=str, choices=["datasets", "pipelines"])

    # eval
    eval = subparsers.add_parser("run", help="Run a pipeline on a specific dataset")
    by_hand = eval.add_argument_group("Manually specify options")
    by_hand.add_argument(
        "-d", "--datasets", type=str, nargs="+", help="Dataset(s) to run on"
    )
    by_hand.add_argument(
        "-p", "--pipeline", type=str, nargs="+", help="Pipeline(s) to run"
    )
    by_hand.add_argument("-o", "--output", type=Path, help="Output directory")
    from_file = eval.add_argument_group("Load config from a file")
    from_file.add_argument("-c", "--config", type=Path, help="Path to a config file")

    # parse
    args = args.parse_args()
    if args.command == "ls":
        ls(args.options)

    elif args.command == "download":
        download_datasets(args.dataset)

    elif args.command == "run":
        if args.config and (args.datasets or args.pipeline or args.output):
            raise ValueError("Cannot specify both config file and manual options")
        # TODO: implement!
        pipelines, datasets, out = parse_config(args.config)
        print(pipelines)
        print(datasets)
        print("Not implemented yet")


if __name__ == "__main__":
    main()
