from .parser import get_datasets


def download_datasets(datasets):
    # parse all datasets
    all_datasets = get_datasets()
    valid_datasets = []
    for d in datasets:
        name, seq = d.split("/")

        # Make sure dataset exists
        DatasetType = all_datasets.get(name, None)
        if DatasetType is None:
            raise ValueError(f"Dataset {name} not found")
            continue

        # Make sure sequence exists
        if seq == "*":
            for seq in DatasetType.sequences():
                valid_datasets.append((DatasetType, seq))
        elif seq in DatasetType.sequences():
            valid_datasets.append((DatasetType, seq))
        elif seq in DatasetType.nicksequences():
            valid_datasets.append((DatasetType, DatasetType.process_seq(seq)))
        else:
            raise ValueError(f"Sequence {seq} not found in {name}")

    # Check if already downloaded
    to_download = []
    for DatasetType, seq in valid_datasets:
        if DatasetType.check_download(seq):
            print(f"Skipping download for {DatasetType.name()}/{seq}, already exists")
        else:
            to_download.append((DatasetType, seq))

    # download each dataset
    print("Will download: ")
    for DatasetType, seq in to_download:
        print(f"  {DatasetType.name()}/{seq}")
    print()

    for DatasetType, seq in valid_datasets:
        print(f"---------- Beginning {DatasetType.name()}/{seq} ----------")
        DatasetType.download(seq)
        print(f"---------- Finished {DatasetType.name()}/{seq} ----------")
