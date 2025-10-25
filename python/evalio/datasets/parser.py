import importlib
from inspect import isclass
import itertools
from types import ModuleType
from typing import NotRequired, Optional, Sequence, TypedDict, cast

from evalio import datasets
from evalio.datasets.base import Dataset
from evalio.utils import CustomException

_DATASETS: set[type[Dataset]] = set()


class DatasetNotFound(CustomException):
    """Exception raised when a dataset is not found."""

    def __init__(self, name: str):
        super().__init__(f"Dataset '{name}' not found")
        self.name = name


class SequenceNotFound(CustomException):
    """Exception raised when a sequence is not found."""

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
) -> int | ImportError:
    """Register a dataset.

    Args:
        dataset (Optional[type[Dataset]], optional): The dataset class to register. Defaults to None.
        module (Optional[ModuleType  |  str], optional): The module containing datasets to register. Defaults to None.

    Returns:
        The number of datasets registered or an ImportError.
    """
    global _DATASETS

    total = 0
    if module is not None:
        if isinstance(module, str):
            try:
                module = importlib.import_module(module)
            except ImportError as e:
                return e

        new_ds = _search_module(module)
        _DATASETS.update(new_ds)
        total += len(new_ds)

    if dataset is not None and _is_dataset(dataset):
        _DATASETS.add(dataset)
        total += 1

    return total


def all_datasets() -> dict[str, type[Dataset]]:
    """Get all registered datasets.

    Returns:
        A dictionary mapping dataset names to their classes.
    """
    global _DATASETS
    return {d.dataset_name(): d for d in _DATASETS}


def get_dataset(name: str) -> type[Dataset] | DatasetNotFound:
    """Get a registered dataset by name.

    Args:
        name (str): The name of the dataset to retrieve.

    Returns:
        The dataset class if found, or a DatasetNotFound error.
    """
    return all_datasets().get(name, DatasetNotFound(name))


def all_sequences() -> dict[str, Dataset]:
    """Get all sequences from all registered datasets.

    Returns:
        A dictionary mapping sequence names to their dataset classes.
    """
    return {
        seq.full_name: seq for d in all_datasets().values() for seq in d.sequences()
    }


def get_sequence(name: str) -> Dataset | SequenceNotFound:
    """Get a registered sequence by name.

    Args:
        name (str): The name of the sequence to retrieve.

    Returns:
        The dataset object if found, or a SequenceNotFound error.
    """
    return all_sequences().get(name, SequenceNotFound(name))


register_dataset(module=datasets)


# ------------------------- Handle yaml parsing ------------------------- #
class DatasetConfig(TypedDict):
    name: str
    length: NotRequired[Optional[int]]


def parse_config(
    d: str | DatasetConfig | Sequence[str | DatasetConfig],
) -> list[tuple[Dataset, int | None]] | DatasetConfigError:
    name: Optional[str] = None
    length: Optional[int] = None
    # If given a list of values
    if isinstance(d, list):
        results = [parse_config(x) for x in d]
        for r in results:
            if isinstance(r, DatasetConfigError):
                return r
        results = cast(list[list[tuple[Dataset, int | None]]], results)
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

    if name[-2:] == "/*":
        ds_name, _ = name.split("/")
        ds = get_dataset(ds_name)
        if isinstance(ds, DatasetNotFound):
            return ds
        return [(s, length) for s in ds.sequences()]

    ds = get_sequence(name)
    if isinstance(ds, SequenceNotFound):
        return ds
    return [(ds, length)]
