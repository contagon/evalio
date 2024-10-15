#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
from pathlib import Path

import argcomplete
import itertools

from .parser import DatasetBuilder, PipelineBuilder, parse_config


def dataset_completer(**kwargs):
    datasets = DatasetBuilder._all_datasets()
    datasets = itertools.chain.from_iterable(
        [f"{d.name()}/{seq}" for seq in d.sequences()] for d in datasets.values()
    )
    return datasets


def pipeline_completer(**kwargs):
    pipelines = PipelineBuilder._all_pipelines()
    pipelines = [p.name() for p in pipelines.values()]
    return pipelines


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
    ).completer = dataset_completer

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
    ).completer = dataset_completer
    by_hand.add_argument(
        "-p", "--pipeline", type=str, nargs="+", help="Pipeline(s) to run"
    ).completer = pipeline_completer
    by_hand.add_argument("-o", "--output", type=Path, help="Output directory")
    by_hand.add_argument(
        "-l", "--length", type=int, help="Number of scans to process for each dataset"
    )
    run.add_argument(
        "-v",
        "--visualize",
        action="count",
        default=0,
        help="Visualize results. Repeat up to 3 times for more detail",
    )

    # stats
    stats = subparsers.add_parser("stats", help="Compute statistics on experiments")
    stats.add_argument(
        "experiments", type=Path, nargs="+", help="Directory(s) to experiments"
    )
    stats.add_argument("-v", "--visualize", action="store_true")
    stats.add_argument("-s", "--sort", type=str, help="Sort by this key")

    # autocomplete
    argcomplete.autocomplete(args)
    args = args.parse_args()

    # Import these now to spend up argcomplete
    from .download import download_datasets
    from .ls import ls
    from .run import run
    from .stats import eval
    from evalio.rerun import RerunVis, RerunConfig

    # parse
    if args.command == "ls":
        ls(args.options)

    elif args.command == "download":
        download_datasets(args.datasets)

    elif args.command == "run":
        # Parse config file
        pipelines, datasets, out = parse_config(args.config)

        # Parse manually specified options
        manual_pipelines = PipelineBuilder.parse(args.pipeline)
        manual_datasets = DatasetBuilder.parse(args.datasets)
        if args.length:
            for d in manual_datasets:
                d.length = args.length

        out = args.output if args.output else out
        pipelines += manual_pipelines
        datasets += manual_datasets

        if out.suffix == ".csv":
            raise ValueError("Output must be a directory")

        # parse visualizer
        vis = RerunVis(args.visualize, RerunConfig())

        run(pipelines, datasets, out, vis)

    elif args.command == "stats":
        eval(args.experiments, args.visualize, args.sort)


if __name__ == "__main__":
    main()
