from evalio.cli.ls import Kind, ls


def test_ls_pipelines():
    ls(Kind.datasets)
    ls(Kind.pipelines)
