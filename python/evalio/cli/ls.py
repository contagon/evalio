from tabulate import tabulate

from .parser import DatasetBuilder, PipelineBuilder
from typing import Optional


def ls(kind: str, search: Optional[str] = None, quiet: bool = False):
    if kind == "datasets":
        data = [["Name", "Sequences", "Down", "GB", "Link"]]
        for d in DatasetBuilder._all_datasets().values():
            if search is not None and search not in d.dataset_name():
                continue
            seq = "\n".join(d.sequences())
            downloaded = [d(s).is_downloaded() for s in d.sequences()]
            downloaded = "\n".join(["✔" if d else "-" for d in downloaded])
            size = [d(s).size_on_disk() for s in d.sequences()]
            size = "\n".join([f"{s:.0f}" if s is not None else "-" for s in size])
            data.append([d.dataset_name(), seq, downloaded, size, d.url()])

        if len(data) == 1:
            print("No datasets found")
            return

        align = ("center", "right", "center", "center", "center")

        # delete unneeded columns if quiet
        if quiet:
            data = [[d[0], d[-1]] for d in data]
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
        data = [["Name", "Params", "Default", "Link"]]
        for p in PipelineBuilder._all_pipelines().values():
            if search is not None and search not in p.name():
                continue
            params = p.default_params()
            keys = "\n".join(params.keys())
            values = "\n".join([str(v) for v in params.values()])
            data.append([p.name(), keys, values, p.url()])

        if len(data) == 1:
            print("No pipelines found")
            return

        align = ("center", "right", "left", "center")

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
