from .parser import get_datasets, get_pipelines
from tabulate import tabulate


def ls(kind):
    if kind == "datasets":
        data = [["Name", "Nname", "Sequences", "Nseq", "Down", "Link"]]
        for d in get_datasets(False).values():
            seq = "\n".join(d.sequences())
            nickseq = "\n".join(d.nicksequences())
            downloaded = [d.check_download(s) for s in d.sequences()]
            downloaded = "\n".join(["✔" if d else "-" for d in downloaded])
            data.append([d.name(), d.nickname(), seq, nickseq, downloaded, d.url()])
        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=("center", "center", "right", "center", "left", "left"),
            )
        )

    if kind == "pipelines":
        data = [["Name", "Nname", "Params", "Types", "Link"]]
        for p in get_pipelines(False).values():
            params = p.params()
            keys = "\n".join(params.keys())
            values = "\n".join(params.values())
            data.append([p.name(), p.nickname(), keys, values, p.url()])
        print(
            tabulate(
                data,
                headers="firstrow",
                tablefmt="fancy_grid",
                rowalign="center",
                colalign=("center", "center", "right", "left"),
            )
        )
