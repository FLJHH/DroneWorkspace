#!/usr/bin/env python3
"""Generate plain diagnostic plots from recorded experiment CSV files."""

import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def plot_static(root):
    paths = sorted(glob.glob(os.path.join(root, "logs", "static_*.csv")))
    if not paths:
        return
    fig, axes = plt.subplots(3, 2, figsize=(12, 12))
    for path in paths:
        rows = list(csv.DictReader(open(path)))
        label = os.path.splitext(os.path.basename(path))[0]
        t = [number(r, "t") for r in rows]
        axes[0, 0].plot([number(r, "uav_x_gt") for r in rows],
                        [number(r, "uav_y_gt") for r in rows], label=label)
        axes[0, 1].plot(t, [number(r, "horizontal_error_gt") for r in rows], label=label)
        axes[1, 0].plot(t, [number(r, "uav_z_gt") for r in rows], label=label)
        axes[1, 1].plot(t, [number(r, "uav_vz_gt") for r in rows], label=label)
        axes[2, 0].step(t, [1.0 if r["tag_visible"] == "True" else 0.0 for r in rows],
                        where="post", label=label)
    titles = ["UAV x-y trajectory (Gazebo GT)", "Horizontal error (Gazebo GT)",
              "UAV z (Gazebo GT)", "UAV vz (Gazebo GT)",
              "AprilTag visible (visual TF)"]
    labels = [("x [m]", "y [m]"), ("t [s]", "error [m]"),
              ("t [s]", "z [m]"), ("t [s]", "vz [m/s]"),
              ("t [s]", "visible")]
    for ax, title, (xlab, ylab) in zip(axes.flat[:5], titles, labels):
        ax.set_title(title); ax.set_xlabel(xlab); ax.set_ylabel(ylab); ax.grid(True)
    axes[0, 0].legend(fontsize=8)
    axes[2, 1].axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(root, "plots", "static_baseline_diagnostics.png"), dpi=140)
    plt.close(fig)


def plot_vehicle(root):
    path = os.path.join(root, "vehicle_motion_0p5.csv")
    if not os.path.exists(path):
        return
    rows = list(csv.DictReader(open(path)))
    t = [number(r, "t") for r in rows]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].plot([number(r, "x_gt") for r in rows], [number(r, "y_gt") for r in rows])
    axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Vehicle x-y trajectory (Gazebo GT)")
    axes[0, 1].plot(t, [number(r, "speed_gt") for r in rows]); axes[0, 1].axhline(0.5, color="r", linestyle="--")
    axes[0, 1].set(xlabel="t [s]", ylabel="speed [m/s]", title="Actual speed")
    axes[1, 0].plot(t, [number(r, "x_gt") for r in rows])
    axes[1, 0].set(xlabel="t [s]", ylabel="x [m]", title="Vehicle x")
    axes[1, 1].plot(t, [number(r, "y_gt") for r in rows])
    axes[1, 1].set(xlabel="t [s]", ylabel="y [m]", title="Vehicle y")
    for ax in axes.flat: ax.grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(root, "plots", "vehicle_motion_0p5_diagnostics.png"), dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    experiment_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "experiments"))
    os.makedirs(os.path.join(experiment_root, "plots"), exist_ok=True)
    plot_static(experiment_root)
    plot_vehicle(experiment_root)
