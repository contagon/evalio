# hack to make matplotlib work with uv
# https://github.com/astral-sh/uv/issues/7036#issuecomment-2416063312
from sys import base_prefix
from os import environ
from pathlib import Path

environ["TCL_LIBRARY"] = str(Path(base_prefix) / "lib" / "tcl8.6")
environ["TK_LIBRARY"] = str(Path(base_prefix) / "lib" / "tk8.6")

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from typing import Callable


Metric = Callable[[np.ndarray, np.ndarray], np.ndarray]


def make_metric_additive(lambda_: float) -> Metric:
    def metric(point: np.ndarray, intensity: np.ndarray) -> np.ndarray:
        return (1.0 - lambda_) * point**2 + lambda_ * intensity**2

    return metric


def make_metric_multiplicative(lambda_: float) -> Metric:
    def multiplicative_metric(point: np.ndarray, intensity: np.ndarray) -> np.ndarray:
        return point**2 * np.abs(intensity) ** lambda_

    return multiplicative_metric


def plot_ax(m: Metric, ax):
    range_point = np.linspace(0, 1)
    range_intensity = np.linspace(0, 1)
    range_point, range_intensity = np.meshgrid(range_point, range_intensity)

    metric_values = m(range_point, range_intensity)

    ax.pcolor(range_point, range_intensity, metric_values, cmap="magma")


def plot_figure(metrics: list[tuple[Metric, str]], rows, cols, filename):
    sns.set_style("whitegrid")

    fig, ax = plt.subplots(
        rows,
        cols,
        layout="constrained",
        sharex=True,
        sharey=True,
        figsize=(2 * cols, 2 * rows),
        squeeze=False,
    )

    for i in range(rows):
        ax[i, 0].set_ylabel(r"$|I_i - I_j|$")
    for j in range(cols):
        ax[-1, j].set_xlabel(r"$|p_i - p_j|$")

    ax = ax.flatten()

    for i, (m, t) in enumerate(metrics):
        plot_ax(m, ax[i])
        ax[i].set_title(t)

    plt.savefig(filename)
    fig.clear()


if __name__ == "__main__":
    # Make additive plot
    rows = 1
    cols = 5
    metrics = [
        (make_metric_additive(lamd), rf"$\lambda = {lamd:.2f}$")
        for lamd in np.linspace(0, 1, rows * cols)
    ]
    plot_figure(metrics, rows, cols, "scripts/additive.png")

    # Make multiplicative plot
    rows = 1
    cols = 5
    metrics = [
        (make_metric_multiplicative(lamd), rf"$p = {lamd:.2f}$")
        for lamd in np.linspace(0, 2, rows * cols)
    ]
    plot_figure(metrics, rows, cols, "scripts/multiplicative.png")

    # Make custom plot
    def custom(p: np.ndarray, i: np.ndarray) -> np.ndarray:
        mask = i < 0.1
        out = np.zeros_like(p)
        out[mask] = p[mask] ** 2
        out[~mask] = p[~mask] ** 2 * 2
        return out

    plot_figure([(custom, "custom")], 1, 1, "scripts/custom.png")
