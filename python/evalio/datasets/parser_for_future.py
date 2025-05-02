from inspect import isclass
from types import ModuleType
from evalio import datasets
from evalio.datasets.base import Dataset
import importlib
import itertools
from typing import Optional


_MODULES_DATASETS = [datasets]


# ------------------------- Overall ------------------------- #
def _search_module(module: ModuleType) -> list[type[Dataset]]:
    return [
        c
        for c in module.__dict__.values()
        if isclass(c) and issubclass(c, Dataset) and c.__name__ != Dataset.__name__
    ]


# ------------------------- Dataset ------------------------- #
def all_datasets() -> list[type[Dataset]]:
    global _MODULES_DATASETS
    datasets = []
    for module in _MODULES_DATASETS:
        datasets.extend(_search_module(module))
    return datasets


def all_sequences() -> list[Dataset]:
    return list(
        itertools.chain.from_iterable(
            [seq for seq in d.sequences()] for d in all_datasets()
        )
    )


def all_sequences_names() -> list[str]:
    return [seq.full_name for seq in all_sequences()]


def all_sequences_dict() -> dict[str, Dataset]:
    return {seq.full_name: seq for seq in all_sequences()}


def get_sequence(name: str) -> Optional[Dataset]:
    return all_sequences_dict().get(name, None)


# ------------------------- Pipeline ------------------------- #
def add_custom_module(module: ModuleType):
    global _MODULES_DATASETS
    if len(_search_module(module)) > 0:
        _MODULES_DATASETS.append(module)
    else:
        raise ValueError(
            f"Module {module.__name__} does not contain any datasets or pipelines"
        )


def add_custom_module_str(module_name: str):
    try:
        module = importlib.import_module(module_name)
    except ImportError:
        raise ValueError(f"Failed to import '{module_name}'")
    add_custom_module(module)
