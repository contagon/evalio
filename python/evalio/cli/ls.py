from tabulate import tabulate

from .parser import DatasetBuilder, PipelineBuilder
from typing import Optional


def ls(kind: str, search: Optional[str] = None, quiet: bool = False):
    if kind == "datasets":
        data = [["Name", "Sequences", "Down", "Link"]]
        for d in DatasetBuilder._all_datasets().values():
            if search is not None and search not in d.dataset_name():
                continue
            seq = "\n".join(d.sequences())
            downloaded = [d(s).is_downloaded() for s in d.sequences()]
            downloaded = "\n".join(["✔" if d else "-" for d in downloaded])
            data.append([d.dataset_name(), seq, downloaded, d.url()])

        if len(data) == 1:
            print("No datasets found")
            return

        align = ("center", "right", "center", "center")

        # delete unneeded columns if quiet
        if quiet:
            data = [[d[0], d[3]] for d in data]
            align = ("center", "center")

        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=align,
            )
        )

    if kind == "pipelines":
        data = [["Name", "Version", "Params", "Default", "Link"]]
        for p in PipelineBuilder._all_pipelines().values():
            if search is not None and search not in p.name():
                continue
            params = p.default_params()
            keys = "\n".join(params.keys())
            values = "\n".join([str(v) for v in params.values()])
            data.append([p.name(), p.version(), keys, values, p.url()])

        if len(data) == 1:
            print("No pipelines found")
            return

        align = ("center", "center", "right", "left", "center")

        # delete unneeded columns if quiet
        if quiet:
            data = [[d[0], d[1], d[-1]] for d in data]
            align = ("center", "center", "center")

        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=align,
            )
        )
