#!/usr/bin/env python3
"""Plot the LiDAR observation-matrix (H^T H) eigenvalues logged by Point-LIO.

Usage:
    python3 plot_eig.py [path/to/eig_log.txt]

The log file is written to <ROOT_DIR>/Log/eig_log.txt when `runtime_eig_log_enable`
is true. Each row is:

    time  t0 t1 t2  r0 r1 r2  f0 f1 f2 f3 f4 f5  effct_feat_num

where t*/r* are the 3 translation / 3 rotation block eigenvalues (ascending) and
f* the full 6x6 eigenvalues (ascending). A near-zero eigenvalue means the pose is
unobservable in that direction -> degenerate scene.
"""
import sys

import numpy as np
import matplotlib.pyplot as plt

# Eigenvalues this many times smaller than the largest are counted as "degenerate".
DEG_THRESH = 1e-2


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 13:
                continue
            rows.append([float(p) for p in parts])
    return np.asarray(rows)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "Log/eig_log.txt"
    data = load(path)
    if data.shape[0] == 0:
        sys.exit(f"no data rows found in {path}")

    t = data[:, 0]
    eig_t = data[:, 1:4]    # translation, ascending
    eig_r = data[:, 4:7]    # rotation, ascending
    eig_f = data[:, 7:13]   # full 6x6, ascending
    feat = data[:, 13] if data.shape[1] > 13 else None

    fig, axes = plt.subplots(3, 2, figsize=(16, 13), sharex=True)

    def plot_block(ax, eig, title, colors):
        for i in range(eig.shape[1]):
            ax.plot(t, np.maximum(eig[:, i], 1e-12), label=f"$\\lambda_{i}$",
                    color=colors[i], linewidth=0.8)
        ax.set_yscale("log")
        ax.set_ylabel(r"eigenvalue of $H^T H$")
        ax.set_title(title)
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, which="both", alpha=0.3)

    # row 1: split translation / rotation blocks
    plot_block(axes[0, 0], eig_t, "Translation block eigenvalues (ascending)",
               ["tab:blue", "tab:cyan", "tab:purple"])
    plot_block(axes[0, 1], eig_r, "Rotation block eigenvalues (ascending)",
               ["tab:orange", "tab:red", "tab:brown"])

    # row 2: full 6x6 (cross-coupling kept) | degeneracy factor
    plot_block(axes[1, 0], eig_f, "Full 6x6 eigenvalues (ascending, cross-coupling kept)",
               ["tab:blue", "tab:cyan", "tab:purple", "tab:orange", "tab:red", "tab:brown"])

    ax = axes[1, 1]
    deg_t = eig_t[:, 0] / np.maximum(eig_t[:, -1], 1e-12)
    deg_r = eig_r[:, 0] / np.maximum(eig_r[:, -1], 1e-12)
    deg_f = eig_f[:, 0] / np.maximum(eig_f[:, -1], 1e-12)
    ax.plot(t, deg_t, color="tab:blue", linewidth=0.8, label="translation")
    ax.plot(t, deg_r, color="tab:orange", linewidth=0.8, label="rotation")
    ax.plot(t, deg_f, color="tab:green", linewidth=0.8, label="full 6x6")
    ax.set_yscale("log")
    ax.set_ylabel(r"$\lambda_{min}/\lambda_{max}$")
    ax.set_title("Degeneracy factor")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, which="both", alpha=0.3)

    # row 3: effective feature count | number of degenerate directions
    ax = axes[2, 0]
    if feat is not None:
        ax.plot(t, feat, color="tab:green", linewidth=0.8)
        ax.set_ylabel("effct_feat_num")
        ax.set_title("Effective plane features per scan")
        ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    # how many of the 6 full eigenvalues are DEG_THRESH below the largest one
    deg_count = (eig_f / np.maximum(eig_f[:, -1:], 1e-12) < DEG_THRESH).sum(axis=1)
    ax.step(t, deg_count, where="post", color="tab:red", linewidth=0.8)
    ax.set_ylim(-0.2, 6.2)
    ax.set_yticks(range(7))
    ax.set_ylabel("# degenerate dirs")
    ax.set_title(f"Degenerate DOF count (full 6x6, $\\lambda_i/\\lambda_{{max}} < {DEG_THRESH:g}$)")
    ax.grid(True, alpha=0.3)

    for ax in axes[-1]:
        ax.set_xlabel("time (s, relative to first lidar frame)")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
