from .base import Dataset, DatasetIterator, get_data_dir, set_data_dir
from .loaders import RawDataIter, RosbagIter

from .boreas import Boreas

from .boreas_rt import BoreasRT
from .botanic_garden import BotanicGarden
from .cumulti import CUMulti
from .enwide import EnWide
from .fomo import Fomo
from .helipr import HeLiPR
from .hilti_2022 import Hilti2022
from .multi_campus import MultiCampus
from .newer_college_2020 import NewerCollege2020
from .newer_college_2021 import NewerCollege2021
from .oxford_spires import OxfordSpires

from .parser import (
    all_datasets,
    get_dataset,
    all_sequences,
    get_sequence,
    register_dataset,
    parse_config,
    DatasetNotFound,
    SequenceNotFound,
    InvalidDatasetConfig,
    DatasetConfigError,
    DatasetConfig,
)

__all__ = [
    # base imports
    "Dataset",
    "DatasetIterator",
    "get_data_dir",
    "set_data_dir",
    # loaders
    "RawDataIter",
    "RosbagIter",
    # parser
    "all_datasets",
    "get_dataset",
    "all_sequences",
    "get_sequence",
    "register_dataset",
    "parse_config",
    "DatasetNotFound",
    "SequenceNotFound",
    "InvalidDatasetConfig",
    "DatasetConfigError",
    "DatasetConfig",
    # datasets
    "Boreas",
    "BoreasRT",
    "BotanicGarden",
    "CUMulti",
    "EnWide",
    "Fomo",
    "HeLiPR",
    "Hilti2022",
    "NewerCollege2020",
    "NewerCollege2021",
    "MultiCampus",
    "OxfordSpires",
]
