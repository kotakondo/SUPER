#!/usr/bin/env python3
"""
Update the SUPER row in the standardized benchmark LaTeX table.

Reads the super_benchmark.csv produced by local_traj_benchmark and updates
the SUPER row in the existing LaTeX table file.

Usage:
    python3 update_standardized_table.py \
        --csv /path/to/super_benchmark.csv \
        --latex-path /path/to/standardized_benchmark.tex
"""

import argparse
import csv
import os
import re
import sys


def compute_stats(csv_path):
    """Read the CSV and compute aggregate statistics over successful cases."""
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    total = len(rows)
    successful = [r for r in rows if int(r["success"]) == 1]
    n_success = len(successful)

    if total == 0:
        print("ERROR: CSV is empty.")
        sys.exit(1)

    success_rate = 100.0 * n_success / total

    if n_success == 0:
        print("WARNING: No successful cases.")
        return {
            "success_rate": success_rate,
            "per_opt_ms": 0.0,
            "total_opt_ms": 0.0,
            "trav_time_s": 0.0,
            "path_length_m": 0.0,
            "jerk_smoothness": 0.0,
            "corridor_viol_pct": 0.0,
            "vel_viol_pct": 0.0,
            "acc_viol_pct": 0.0,
            "jerk_viol_pct": 0.0,
        }

    avg = lambda key: sum(float(r[key]) for r in successful) / n_success

    per_opt = avg("per_opt_runtime_ms")
    total_opt = avg("total_opt_runtime_ms")
    trav_time = avg("total_traj_time_sec")
    path_length = avg("traj_length_m")
    jerk_smooth = avg("jerk_smoothness_l1")

    # Corridor violation: average sample-point violation % across cases
    corridor_viol_pct = avg("corridor_violation_pct")

    # Dynamic violations: average sample-point violation % across cases
    vel_viol_pct = avg("v_violation_pct")
    acc_viol_pct = avg("a_violation_pct")
    jerk_viol_pct = avg("j_violation_pct")

    return {
        "success_rate": success_rate,
        "per_opt_ms": per_opt,
        "total_opt_ms": total_opt,
        "trav_time_s": trav_time,
        "path_length_m": path_length,
        "jerk_smoothness": jerk_smooth,
        "corridor_viol_pct": corridor_viol_pct,
        "vel_viol_pct": vel_viol_pct,
        "acc_viol_pct": acc_viol_pct,
        "jerk_viol_pct": jerk_viol_pct,
    }


def fmt(val, decimals=1):
    """Format a float to the given number of decimal places."""
    return f"{val:.{decimals}f}"


def build_super_row(stats):
    """Build the SUPER row LaTeX string.

    SUPER uses a single optimization call, so per_opt == total_opt.
    We merge them with \\multicolumn{2}{c}{value}.
    """
    opt_time = fmt(stats["per_opt_ms"])
    row = (
        f"      SUPER & multi & -- "
        f"& {fmt(stats['success_rate'])} "
        f"& \\multicolumn{{2}}{{c}}{{{opt_time}}} "
        f"& {fmt(stats['trav_time_s'])} "
        f"& {fmt(stats['path_length_m'])} "
        f"& {fmt(stats['jerk_smoothness'])} "
        f"& {fmt(stats['corridor_viol_pct'])} "
        f"& {fmt(stats['vel_viol_pct'])} "
        f"& {fmt(stats['acc_viol_pct'])} "
        f"& {fmt(stats['jerk_viol_pct'])} \\\\"
    )
    return row


def update_table(latex_path, new_row):
    """Find and replace the SUPER row in the LaTeX table."""
    with open(latex_path, "r") as f:
        lines = f.readlines()

    found = False
    for i, line in enumerate(lines):
        stripped = line.strip()
        # Match line starting with "SUPER" (possibly with \best/\worst wrappers)
        if re.match(r"^\s*SUPER\s*&", line):
            lines[i] = new_row + "\n"
            found = True
            break

    if not found:
        print(f"ERROR: Could not find SUPER row in {latex_path}")
        sys.exit(1)

    with open(latex_path, "w") as f:
        f.writelines(lines)

    print(f"Updated SUPER row in {latex_path}")


def main():
    parser = argparse.ArgumentParser(description="Update SUPER row in standardized benchmark table")
    parser.add_argument("--csv", required=True, help="Path to super_benchmark.csv")
    parser.add_argument("--latex-path", required=True, help="Path to standardized_benchmark.tex")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"ERROR: CSV not found: {args.csv}")
        sys.exit(1)

    if not os.path.exists(args.latex_path):
        print(f"ERROR: LaTeX file not found: {args.latex_path}")
        sys.exit(1)

    stats = compute_stats(args.csv)

    print("=== SUPER Standardized Benchmark Stats ===")
    print(f"  Success rate:    {fmt(stats['success_rate'])}%")
    print(f"  Per-opt time:    {fmt(stats['per_opt_ms'])} ms")
    print(f"  Total-opt time:  {fmt(stats['total_opt_ms'])} ms")
    print(f"  Travel time:     {fmt(stats['trav_time_s'])} s")
    print(f"  Path length:     {fmt(stats['path_length_m'])} m")
    print(f"  Jerk smoothness: {fmt(stats['jerk_smoothness'])} m/s^2")
    print(f"  Corridor viol:   {fmt(stats['corridor_viol_pct'])}%")
    print(f"  Vel viol:        {fmt(stats['vel_viol_pct'])}%")
    print(f"  Acc viol:        {fmt(stats['acc_viol_pct'])}%")
    print(f"  Jerk viol:       {fmt(stats['jerk_viol_pct'])}%")

    new_row = build_super_row(stats)
    print(f"\nNew row:\n  {new_row}")

    update_table(args.latex_path, new_row)


if __name__ == "__main__":
    main()
