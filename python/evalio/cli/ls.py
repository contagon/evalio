from tabulate import tabulate

from .parser import DatasetBuilder, PipelineBuilder


def ls(kind):
    if kind == "datasets":
        data = [["Name", "Sequences", "Down", "Link"]]
        for d in DatasetBuilder._all_datasets().values():
            seq = "\n".join(d.sequences())
            downloaded = [d(s).is_downloaded() for s in d.sequences()]
            downloaded = "\n".join(["✔" if d else "-" for d in downloaded])
            data.append([d.dataset_name(), seq, downloaded, d.url()])
        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=("center", "right", "left", "left"),
            )
        )

    if kind == "pipelines":
        data = [["Name", "Params", "Default", "Link"]]
        for p in PipelineBuilder._all_pipelines().values():
            params = p.default_params()
            keys = "\n".join(params.keys())
            values = "\n".join([str(v) for v in params.values()])
            data.append([p.name(), keys, values, p.url()])
        if len(data) == 1:
            print("No pipelines found")
            return
        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=("center", "right", "left", "center"),
            )
        )
