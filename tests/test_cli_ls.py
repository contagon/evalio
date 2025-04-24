from evalio.cli.ls import ls, Kind
from evalio.cli.parser import DatasetBuilder


def test_ls_pipelines():
    ls(Kind.datasets)
    ls(Kind.pipelines)


def test_dataset_build():
    all_datasets, all_names = map(
        list,
        zip(
            *[
                (s, s.full_name)
                for d in DatasetBuilder._all_datasets().values()
                for s in d.sequences()
            ]
        ),
    )

    # Test all datasets
    out = [d.dataset for d in DatasetBuilder.parse(all_names)]
    if out != all_datasets:
        assert len(out) == len(all_datasets), f"Missing datasets in parser {all_names}"
        for d, name in zip(out, all_names):
            assert out == d, f"Failed on {name}"
