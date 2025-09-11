from evalio.cli.parser import DatasetBuilder
from rich import print
from evalio.utils import print_warning

# Helper script to get the lengths of all datasets
# Easier than manually checking each
for d in DatasetBuilder.all_datasets().values():
    lengths = {}

    for seq in d.sequences():
        if not seq.is_downloaded():
            print_warning(f"Sequence {seq.name} not downloaded")
            continue

        try:
            lengths[seq.name] = len(seq.data_iter())
        except Exception:
            print_warning(f"Failed to get length of {seq.name}")
            continue

    print(d.dataset_name())
    print(lengths)
    print()
