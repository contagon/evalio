import argparse
from pathlib import Path

from .download import download_datasets
from .parser import parse_config, parse_datasets, parse_pipelines
from .ls import ls
from .run import run


def main():
    args = argparse.ArgumentParser(
        "Tool for evaluating Lidar-Inertial Odometry pipelines on open-source datasets"
    )
    subparsers = args.add_subparsers(dest="command")

    # downloading
    download = subparsers.add_parser("download", help="Download datasets")
    download.add_argument(
        "datasets", type=str, help="Dataset(s) to download", nargs="+"
    )

    # ls
    ls_opt = subparsers.add_parser("ls", help="List available datasets and pipelines")
    ls_opt.add_argument("options", type=str, choices=["datasets", "pipelines"])

    # eval
    eval = subparsers.add_parser("run", help="Run a pipeline on a specific dataset")
    by_hand = eval.add_argument_group("Manually specify options")
    by_hand.add_argument(
        "-d",
        "--datasets",
        type=str,
        nargs="+",
        required=True,
        help="Dataset(s) to run on",
    )
    by_hand.add_argument(
        "-p",
        "--pipeline",
        type=str,
        nargs="+",
        required=True,
        help="Pipeline(s) to run",
    )
    by_hand.add_argument(
        "-o", "--output", type=Path, required=True, help="Output directory"
    )
    by_hand.add_argument(
        "-l", "--length", type=int, help="Number of scans to process for each dataset"
    )
    from_file = eval.add_argument_group("Load config from a file")
    from_file.add_argument("-c", "--config", type=Path, help="Path to a config file")
    eval.add_argument("-v", "--visualize", action="store_true")

    # parse
    args = args.parse_args()
    if args.command == "ls":
        ls(args.options)

    elif args.command == "download":
        download_datasets(args.datasets)

    elif args.command == "run":
        if args.config and (args.datasets or args.pipeline or args.output):
            raise ValueError("Cannot specify both config file and manual options")
        if args.config:
            pipelines, datasets, out = parse_config(args.config)
        else:
            pipelines = parse_pipelines(args.pipeline)
            if args.length:
                datasets = parse_datasets(
                    [{"name": d, "length": args.length} for d in args.datasets]
                )
            else:
                datasets = parse_datasets(args.datasets)
            out = args.output

        run(pipelines, datasets, out, visualize=args.visualize)


if __name__ == "__main__":
    main()
