import importlib
from inspect import isclass
import itertools
from types import ModuleType
from typing import Callable, NotRequired, Optional, Sequence, TypedDict, cast

from evalio import datasets
from evalio.datasets.base import Dataset
from evalio.utils import CustomException

_DATASETS: set[type[Dataset]] = set()


class DatasetNotFound(CustomException):
    def __init__(self, name: str):
        super().__init__(f"Dataset '{name}' not found")
        self.name = name


class SequenceNotFound(CustomException):
    def __init__(self, name: str):
        super().__init__(f"Sequence '{name}' not found")
        self.name = name


class InvalidDatasetConfig(CustomException):
    def __init__(self, config: str):
        super().__init__(f"Invalid config: '{config}'")
        self.config = config


DatasetConfigError = DatasetNotFound | SequenceNotFound | InvalidDatasetConfig


# ------------------------- Handle Registration of Datasets ------------------------- #
def _is_dataset(obj: object) -> bool:
    return (
        isclass(obj) and issubclass(obj, Dataset) and obj.__name__ != Dataset.__name__
    )


def _search_module(module: ModuleType) -> set[type[Dataset]]:
    return {c for c in module.__dict__.values() if _is_dataset(c)}


def register_dataset(
    dataset: Optional[type[Dataset]] = None,
    module: Optional[ModuleType | str] = None,
):
    global _DATASETS

    if module is not None:
        if isinstance(module, str):
            try:
                module = importlib.import_module(module)
            except ImportError:
                raise ValueError(f"Failed to import '{module}'")

        if len(new_ds := _search_module(module)) > 0:
            _DATASETS.update(new_ds)
        else:
            raise ValueError(
                f"Module {module.__name__} does not contain any datasets or pipelines"
            )

    if dataset is not None:
        if _is_dataset(dataset):
            _DATASETS.add(dataset)
        else:
            raise ValueError(f"{dataset} is not a valid Dataset subclass")


def all_datasets() -> dict[str, type[Dataset]]:
    global _DATASETS
    return {d.dataset_name(): d for d in _DATASETS}


def get_dataset(name: str) -> type[Dataset] | DatasetNotFound:
    return all_datasets().get(name, DatasetNotFound(name))


def all_sequences() -> dict[str, Dataset]:
    return {
        seq.full_name: seq for d in all_datasets().values() for seq in d.sequences()
    }


def get_sequence(name: str) -> Dataset | SequenceNotFound:
    return all_sequences().get(name, SequenceNotFound(name))


register_dataset(module=datasets)


# ------------------------- Handle yaml parsing ------------------------- #
class DatasetConfig(TypedDict):
    name: str
    length: NotRequired[Optional[int]]


def parse_config(
    d: str | DatasetConfig | Sequence[str | DatasetConfig],
) -> list[tuple[Dataset, int]] | DatasetConfigError:
    name: Optional[str] = None
    length: Optional[int] = None
    # If given a list of values
    if isinstance(d, list):
        results = [parse_config(x) for x in d]
        for r in results:
            if isinstance(r, DatasetConfigError):
                return r
        results = cast(list[list[tuple[Dataset, int]]], results)
        return list(itertools.chain.from_iterable(results))

    # If it's a single config
    elif isinstance(d, str):
        name = d
        length = None
    elif isinstance(d, dict):
        name = d.get("name", None)
        length = d.get("length", None)
    else:
        return InvalidDatasetConfig(str(d))

    if name is None:  # type: ignore
        return InvalidDatasetConfig("Missing 'name' in dataset config")

    length_lambda: Callable[[Dataset], int]
    if length is None:
        length_lambda = lambda s: len(s)
    else:
        length_lambda = lambda s: length

    if name[-2:] == "/*":
        ds_name, _ = name.split("/")
        ds = get_dataset(ds_name)
        if isinstance(ds, DatasetNotFound):
            return ds
        return [(s, length_lambda(s)) for s in ds.sequences()]

    ds = get_sequence(name)
    if isinstance(ds, SequenceNotFound):
        return ds
    return [(ds, length_lambda(ds))]
