#!/usr/bin/env python3
import os
import sys
import glob
import csv
import math

class RunningStats:
    """Welford’s algorithm for mean, variance, plus min/max tracking."""
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0
        self.min = float('inf')
        self.max = float('-inf')

    def add(self, x):
        self.n += 1
        if x < self.min: self.min = x
        if x > self.max: self.max = x
        delta = x - self.mean
        self.mean += delta / self.n
        self.M2 += delta * (x - self.mean)

    @property
    def avg(self):
        return self.mean if self.n > 0 else 0.0

    @property
    def std(self):
        return math.sqrt(self.M2 / (self.n - 1)) if self.n > 1 else 0.0

def compute_file_stats(filepath, track_cols, global_stats):
    """
    Compute per-file averages for all cols in track_cols ∪ {EXP_SUCCESS},
    up‑dating global_stats (Welford) row-by-row.
    """
    sums = {col: 0.0 for col in track_cols | {"EXP_SUCCESS"}}
    counts = {col: 0   for col in track_cols | {"EXP_SUCCESS"}}

    with open(filepath, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                succ = int(row["EXP_SUCCESS"].strip())
            except (KeyError, ValueError):
                continue
            if succ == -1:
                continue

            # always include EXP_SUCCESS
            sums["EXP_SUCCESS"] += succ
            counts["EXP_SUCCESS"] += 1
            global_stats["EXP_SUCCESS"].add(succ)

            if succ == 1:
                for col in track_cols:
                    if col in row:
                        try:
                            v = float(row[col].strip())
                        except ValueError:
                            continue
                        sums[col] += v
                        counts[col] += 1
                        global_stats[col].add(v)

    # per-file averages
    file_avgs = {}
    for col in sums:
        if counts[col] > 0:
            file_avgs[col] = (counts[col], sums[col] / counts[col])
        else:
            file_avgs[col] = (0, 0.0)
    return file_avgs

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <folder_path>")
        sys.exit(1)

    folder = sys.argv[1]
    pattern = os.path.join(folder, "time_consuming_num_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print("No files found matching pattern in folder:", folder)
        sys.exit(1)

    # columns to track beyond EXP_SUCCESS
    track_cols = {
        "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
        "EPX_TRAJ_FRONTEND","EPX_TRAJ_GLOBAL",
        "EXP_TRAJ_INIT", "EXP_TRAJ_OPT",
        "GENERATE_BACK_TRAJ","GENERATE_EXP_TRAJ",
        "TIME_STAMPE", "TOTAL_REPLAN","VISUALIZATION"
    }

    # initialize global Welford stats
    global_stats = {col: RunningStats() for col in track_cols | {"EXP_SUCCESS"}}

    # collect per-file averages
    all_file_avgs = []
    for fp in files:
        avgs = compute_file_stats(fp, track_cols, global_stats)
        all_file_avgs.append((os.path.basename(fp), avgs))

    # write to output file
    out_path = os.path.join(folder, "average_computation_times.txt")
    with open(out_path, "w") as f:
        for filename, avgs in all_file_avgs:
            f.write(f"File: {filename}\n")
            for col in sorted(avgs):
                n, a = avgs[col]
                f.write(f"   {col} (n={n}): {a:.6f}\n")
            f.write("\n")

        f.write("Overall Averages:\n")
        f.write("====================================\n")
        # overall averages (count & mean) for all columns except EXP_TRAJ_OPT
        for col in sorted(global_stats):
            rs = global_stats[col]
            if col == "EXP_TRAJ_OPT":
                # full stats for EXP_TRAJ_OPT
                f.write(f"   {col}: n={rs.n}, avg={rs.avg:.6f}, "
                        f"min={rs.min:.6f}, max={rs.max:.6f}, std={rs.std:.6f}\n")
            else:
                # only avg for others
                f.write(f"   {col}: {rs.avg:.6f}\n")

    print("Results written to", out_path)

if __name__ == "__main__":
    main()
