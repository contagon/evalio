import evalio


def find_types(module, skip=None):
    found = {}
    # Include by name
    found |= dict(
        (cls.name(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    # Include by nickname
    found |= dict(
        (cls.nickname(), cls)
        for cls in module.__dict__.values()
        if isinstance(cls, type) and cls.__name__ != skip.__name__
    )
    return found


def get_datasets():
    return find_types(evalio.datasets, skip=evalio.datasets.Dataset)


def get_pipelines():
    return find_types(evalio.pipelines, skip=evalio.pipelines.Pipeline)
