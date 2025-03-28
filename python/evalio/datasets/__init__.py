from .base import Dataset, DatasetIterator, set_data_dir, get_data_dir
from .botanic_garden import BotanicGarden
from .enwide import EnWide
from .helipr import HeLiPR
from .hilti_2022 import Hilti2022
from .newer_college_2020 import NewerCollege2020
from .newer_college_2021 import NewerCollege2021
from .multi_campus import MultiCampus
from .oxford_spires import OxfordSpires

from . import loaders

__all__ = [
    "get_data_dir",
    "set_data_dir",
    "loaders",
    "Dataset",
    "DatasetIterator",
    "BotanicGarden",
    "EnWide",
    "HeLiPR",
    "Hilti2022",
    "NewerCollege2020",
    "NewerCollege2021",
    "MultiCampus",
    "OxfordSpires",
]
