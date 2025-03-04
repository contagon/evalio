# hack to make matplotlib work with uv
# https://github.com/astral-sh/uv/issues/7036#issuecomment-2416063312
from sys import base_prefix
from os import environ
from pathlib import Path

environ["TCL_LIBRARY"] = str(Path(base_prefix) / "lib" / "tcl8.6")
environ["TK_LIBRARY"] = str(Path(base_prefix) / "lib" / "tk8.6")


import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import sys
from pathlib import Path

file = Path(sys.argv[1])
stem = file.stem
dir = file.parent

# ------------------------- Read in and clean ------------------------- #
df = pd.read_csv(file)

# Drop experimental add rows
df = df[df["intensity_metric"] != "add_norm1"]
df = df[df["intensity_metric"] != "add_norm12"]
df = df[df["intensity_metric"] != "add_norm2"]
df = df[df["intensity_residual"] != "add_norm1"]
df = df[df["intensity_residual"] != "add_norm12"]
df = df[df["intensity_residual"] != "add_norm2"]
df = df[df["intensity_residual"] != "add25"]
df = df[df["intensity_residual"] != "add50"]
df = df[df["intensity_residual"] != "add75"]

# ------------------------- Plot ------------------------- #
df = df.pivot(columns="intensity_metric", index="intensity_residual", values="ATEt")
df = df[["norm0", "norm1", "norm2", "add25", "add50", "add75"]]
df.rename(
    index={
        "norm0": "Baseline",
        "norm_m1": r"$1 \;/\; |I_i - I_j|$",
        "norm_m2": r"$1 \;/\; |I_i - I_j|^2$",
    },
    columns={
        "norm0": "Baseline",
        "norm1": r"$|I_i - I_j|$",
        "norm2": r"$|I_i - I_j|^2$",
        "add25": r"$+ 0.25 |I_i - I_j|$",
        "add50": r"$+ 0.50 |I_i - I_j|$",
        "add75": r"$+ 0.75 |I_i - I_j|$",
    },
    inplace=True,
)

fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
sns.heatmap(df, annot=True, fmt=".1f", ax=ax)
ax.tick_params(axis="y", rotation=0)
ax.set_xlabel("Intensity Metric")
ax.set_ylabel("Intensity Residual")
plt.savefig(dir / f"{stem}.png")
