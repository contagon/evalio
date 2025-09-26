import importlib
from inspect import isclass
from types import ModuleType
from typing import Optional

from evalio import datasets
from evalio.datasets.base import Dataset

_DATASETS: set[type[Dataset]] = set()


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


def get_dataset(name: str) -> Optional[type[Dataset]]:
    return all_datasets().get(name, None)


def all_sequences() -> dict[str, Dataset]:
    return {
        seq.full_name: seq for d in all_datasets().values() for seq in d.sequences()
    }


def get_sequence(name: str) -> Optional[Dataset]:
    return all_sequences().get(name, None)


register_dataset(module=datasets)

# ------------------------- Handle yaml parsing ------------------------- #
