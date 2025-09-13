from pathlib import Path

# Helper script to return all files that are included in a dataset
# Easier than manually checking each
dir = Path("/media/contagon/Aquatonomy/evalio/newer_college_2020")
results: dict[str, list[str]] = {}
for seq in dir.iterdir():
    results[seq.stem] = sorted([file.name for file in seq.iterdir()])

print(results)
