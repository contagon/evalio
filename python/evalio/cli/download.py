from .parser import DatasetBuilder


def download_datasets(datasets: list[str]) -> None:
    # parse all datasets
    valid_datasets = DatasetBuilder.parse(datasets)

    # Check if already downloaded
    to_download = []
    for builder in valid_datasets:
        if builder.check_download():
            print(f"Skipping download for {builder}, already exists")
        else:
            to_download.append(builder)

    if len(to_download) == 0:
        print("Nothing to download, finishing")
        return

    # download each dataset
    print("Will download: ")
    for builder in to_download:
        print(f"  {builder}")
    print()

    for builder in to_download:
        print(f"---------- Beginning {builder} ----------")
        builder.download()
        print(f"---------- Finished {builder} ----------")
