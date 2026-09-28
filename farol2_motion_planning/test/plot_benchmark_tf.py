#!/usr/bin/env python3
"""
plot_benchmark_results.py

Reads the CSVs produced by benchmark_planner.cpp and generates the figures
for the book chapter's computational-cost study:

  1. benchmark_degree_1veh_0obs.csv  -> time/failure vs. Bezier degree
                                         (1 vehicle, 0 obstacles)
  2. benchmark_vehicles_deg9_0obs.csv -> time/failure vs. number of vehicles
                                         (Bezier degree fixed at 9, 0 obstacles)

For each sweep, produces three plots:
  - average solve time over successful trials
  - failure rate (%)
  - worst-case (max) solve time over successful trials

Usage:
    python3 plot_benchmark_results.py [--indir DIR] [--outdir DIR]

Requires: pandas, matplotlib
"""

import argparse
import datetime
import os
import pandas as pd
import matplotlib.pyplot as plt


def plot_sweep(csv_path, x_col, x_label, title_prefix, outdir):
    if not os.path.exists(csv_path):
        print(f"[skip] {csv_path} not found")
        return

    df = pd.read_csv(csv_path, comment="#")
    df = df.sort_values(x_col)

    base = os.path.splitext(os.path.basename(csv_path))[0]

    # --- Average solve time (successful trials only) ---
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(df[x_col], df["avg_time_success_ms"], marker="o", ms=3)
    
    ax.set_xlabel(x_label)
    ax.set_ylabel("Solve time [ms]\n")
    #ax.set_title(f"{title_prefix}: average solve time")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"{base}_solve_time.png"), dpi=300)
    plt.close(fig)

    # --- Failure rate ---
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(df[x_col], df["tf_value"], marker="o", ms=3)
    ax.set_xlabel(x_label, fontsize=12)
    ax.set_ylabel("Maneuver Time [s]", fontsize=12)
    ax.axhline(y=5.457865264, linestyle="--", color="firebrick", linewidth=1)
    ax.tick_params(axis="both", labelsize=11)
    # ax.set_ylim(5.4, 6.1)
    ax.grid(True, linewidth=0.4, alpha=1.0)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"{base}_tf_value.png"), dpi=300)
    plt.close(fig)


RESULTS_DIR = "/home/dsor-gorka/dsor/colcon_ws_mDrivers/src/farol2/farol2_motion_planning/test/results"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indir", default=RESULTS_DIR, help="directory containing the benchmark CSVs")
    parser.add_argument("--outdir", default=os.path.join(RESULTS_DIR, "figures"), help="directory to write PNG figures to")
    parser.add_argument(
        "--date",
        default=datetime.date.today().isoformat(),
        help="date tag matching the CSV filenames, format YYYY-MM-DD (default: today)",
    )
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    plot_sweep(
        csv_path=os.path.join(args.indir, f"benchmark_{args.date}_simple.csv"),
        x_col="bezier_degree",
        x_label="Bézier Curve Degree",
        title_prefix="1 vehicle, 0 obstacles",
        outdir=args.outdir,
    )

    # plot_sweep(
    #     csv_path=os.path.join(args.indir, f"benchmark_vehicles_deg9_0obs_{args.date}.csv"),
    #     x_col="n_vehicles",
    #     x_label="Number of vehicles",
    #     title_prefix="Bezier degree = 9, 0 obstacles",
    #     outdir=args.outdir,
    # )


if __name__ == "__main__":
    main()