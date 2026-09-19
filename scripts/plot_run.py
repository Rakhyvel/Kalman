#!/usr/bin/env python3
"""
Plots a run.

Usage:
    python scripts/plot_run.py <path to run csv>
"""

import csv
import sys

import matplotlib.pyplot as plt


def load_data(path):
    t, truth, measurement, estimate = [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            truth.append(float(row["truth"]))
            measurement.append(float(row["measurement"]))
            estimate.append(float(row["estimate"]))
    return t, truth, measurement, estimate


def main():
    if len(sys.argv) != 2:
        print("usage: python plot_run.py <path to csv>")
        return

    path = sys.argv[1]
    t, truth, measurement, estimate = load_data(path)

    fig, ax = plt.subplots(figsize=(9, 5.5))

    ax.plot(t, truth, label="Truth", color="black", linewidth=2, zorder=3)
    ax.scatter(
        t, measurement, label="Measurement", color="tab:red", marker="x", s=40, zorder=2
    )
    ax.plot(
        t,
        estimate,
        label="Estimate",
        color="tab:blue",
        linewidth=2,
        linestyle="--",
        zorder=2,
    )

    ax.set_xlabel("t")
    ax.set_ylabel("value")
    ax.set_title("Truth vs. Measurement vs. Estimate")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
