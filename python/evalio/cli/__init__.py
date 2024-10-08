#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
from pathlib import Path

import argcomplete


def main():
    # TODO: Load in rerun ip from here somehow
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

    # run
    run = subparsers.add_parser("run", help="Run a pipeline on a specific dataset")
    from_file = run.add_argument_group("Load config from a file")
    from_file.add_argument("-c", "--config", type=Path, help="Path to a config file")
    by_hand = run.add_argument_group("Manually specify options")
    by_hand.add_argument(
        "-d", "--datasets", type=str, nargs="+", help="Dataset(s) to run on"
    )
    by_hand.add_argument(
        "-p", "--pipeline", type=str, nargs="+", help="Pipeline(s) to run"
    )
    by_hand.add_argument("-o", "--output", type=Path, help="Output directory")
    by_hand.add_argument(
        "-l", "--length", type=int, help="Number of scans to process for each dataset"
    )
    run.add_argument("-v", "--visualize", action="store_true")

    # stats
    stats = subparsers.add_parser("stats", help="Compute statistics on experiments")
    stats.add_argument("experiments", type=Path, help="Directory to experiments")
    stats.add_argument("-v", "--visualize", action="store_true")

    argcomplete.autocomplete(args)
    args = args.parse_args()

    # Import these now to spend up argcomplete
    from .download import download_datasets
    from .ls import ls
    from .parser import parse_config, parse_datasets, parse_pipelines
    from .run import run
    from .stats import eval

    # parse
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

        if out.suffix != "":
            raise ValueError("Output must be a directory")

        run(pipelines, datasets, out, visualize=args.visualize)

    elif args.command == "stats":
        eval(args.experiments, args.visualize)


if __name__ == "__main__":
    main()
