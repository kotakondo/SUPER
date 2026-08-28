#!/usr/bin/env python3
"""
Update the SUPER rows in the standardized benchmark LaTeX table.

Reads two CSVs (L2 and L-inf) produced by local_traj_benchmark and updates
the two SUPER rows in the existing LaTeX table file.

Usage:
    python3 update_standardized_table.py \
        --csv-l2 /path/to/super_l2_benchmark.csv \
        --csv-linf /path/to/super_linf_benchmark.csv \
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


def build_super_row(stats, norm_label, is_first_row=True):
    """Build a SUPER row LaTeX string.

    Args:
        stats: dict of computed statistics
        norm_label: LaTeX string for the norm column, e.g. "$L_2$" or "$L_\\infty$"
        is_first_row: if True, emit \\multirow{2}{*}{SUPER} and \\multirow{2}{*}{multi}
    """
    algo = "\\multirow{2}{*}{SUPER}" if is_first_row else ""
    thread = "\\multirow{2}{*}{multi}" if is_first_row else ""
    opt_time = fmt(stats["per_opt_ms"])
    row = (
        f"      {algo} & {thread} & {norm_label} "
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


def update_table(latex_path, l2_row, linf_row):
    """Find and replace the two SUPER rows in the LaTeX table."""
    with open(latex_path, "r") as f:
        lines = f.readlines()

    # Find the L2 row (first SUPER row): matches \multirow{2}{*}{SUPER} or lines with SUPER.*L_2
    l2_found = False
    linf_found = False

    for i, line in enumerate(lines):
        # Match L2 row: contains "SUPER" and "L_2"
        if re.search(r"SUPER.*L_2", line) or re.search(r"\\multirow\{2\}\{\*\}\{SUPER\}.*L_2", line):
            lines[i] = l2_row + "\n"
            l2_found = True
        # Match L-inf row: contains "L_\\infty" or "L_\infty"
        elif re.search(r"L_\\\\infty|L_\\infty", line) and not re.search(r"(FASTER|DYNUS)", line):
            lines[i] = linf_row + "\n"
            linf_found = True

    if not l2_found or not linf_found:
        print(f"ERROR: Could not find SUPER rows in {latex_path}")
        print(f"  L2 row found: {l2_found}, L-inf row found: {linf_found}")
        sys.exit(1)

    with open(latex_path, "w") as f:
        f.writelines(lines)

    print(f"Updated SUPER rows (L2 + L-inf) in {latex_path}")


def main():
    parser = argparse.ArgumentParser(description="Update SUPER rows in standardized benchmark table")
    parser.add_argument("--csv-l2", required=True, help="Path to super_l2_benchmark.csv")
    parser.add_argument("--csv-linf", required=True, help="Path to super_linf_benchmark.csv")
    parser.add_argument("--latex-path", required=True, help="Path to standardized_benchmark.tex")
    args = parser.parse_args()

    for path, label in [(args.csv_l2, "L2 CSV"), (args.csv_linf, "L-inf CSV")]:
        if not os.path.exists(path):
            print(f"ERROR: {label} not found: {path}")
            sys.exit(1)

    if not os.path.exists(args.latex_path):
        print(f"ERROR: LaTeX file not found: {args.latex_path}")
        sys.exit(1)

    # Compute stats for each norm
    l2_stats = compute_stats(args.csv_l2)
    linf_stats = compute_stats(args.csv_linf)

    for label, stats in [("L2", l2_stats), ("L-inf", linf_stats)]:
        print(f"\n=== SUPER Standardized Benchmark Stats ({label}) ===")
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

    # Build row strings
    l2_row = build_super_row(l2_stats, "$L_2$", is_first_row=True)
    linf_row = build_super_row(linf_stats, "$L_\\infty$", is_first_row=False)

    print(f"\nNew rows:")
    print(f"  {l2_row}")
    print(f"  {linf_row}")

    update_table(args.latex_path, l2_row, linf_row)


if __name__ == "__main__":
    main()
