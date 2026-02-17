#!/usr/bin/env python3
"""
SUPER Static Benchmark Analyzer

Analyzes benchmark data from SUPER's static environment benchmark and
generates/updates the SUPER row in the static benchmark LaTeX table.

Usage (single environment - inside Docker):
    python3 analyze_static_benchmark.py --data-dir /home/kota/data

Usage (multi-environment - forest benchmark):
    python3 analyze_static_benchmark.py --data-dir /home/kota/data \
        --envs easy_forest,medium_forest,hard_forest

Usage (with custom constraints):
    python3 analyze_static_benchmark.py --data-dir /home/kota/data \
        --v-max 10.0 --a-max 20.0 --j-max 30.0
"""

import argparse
import math
import os
import sys
import glob
import csv
import numpy as np

# Add current directory to path for importing sibling modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_bag import process_bag, extract_number
from compute_average_computation import RunningStats, compute_file_stats


DEFAULT_LATEX_PATH = "/home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex"

# Default robot radius [m] for collision checking (point mass)
ROBOT_RADIUS = 0

# Mapping from env directory name to LaTeX table label
ENV_TO_LABEL = {
    "easy_forest": "Easy",
    "medium_forest": "Medium",
    "hard_forest": "Hard",
}

# Mapping from planner name to LaTeX constraint label
PLANNER_TO_CONSTR_LABEL = {
    "super_l2": "$L_2$",
    "super_linf": "$L_\\infty$",
}

# Default table template (matches the paper format with per-environment rows)
DEFAULT_TABLE_TEMPLATE = r"""\begin{table*}
  \caption{Benchmark results against state-of-the-art methods in static environments. DYNUS outperforms the other methods in terms of travel time and achieves a 100\% success rate. Since SUPER performs global path planning for both exploratory and safe trajectories, we list the corresponding computation times as {Exploratory | Safe} in the Global Path Planning Computation Time column.}
  \label{tab:static_benchmark}
  \centering
  \renewcommand{\arraystretch}{1.2}
  \resizebox{\textwidth}{!}{
    \begin{tabular}{c c c c c c c c c c c c c}
      \toprule
      \multirow{2}{*}[-0.4ex]{\textbf{Env}}
      & \multirow{2}{*}[-0.4ex]{\textbf{Algorithm}}
      & \multicolumn{2}{c}{\multirow{2}{*}[-0.4ex]{\textbf{Constr.}}}
      & \multicolumn{1}{c}{\textbf{Success}}
      & \multicolumn{2}{c}{\textbf{Computation Time}}
      & \multicolumn{3}{c}{\textbf{Performance}}
      & \multicolumn{3}{c}{\textbf{Constraint Violation}}
      \\
      \cmidrule(lr){5-5}
      \cmidrule(lr){6-7}
      \cmidrule(lr){8-10}
      \cmidrule(lr){11-13}
      &&&&
      $R_{\mathrm{succ}}$ [\%]
      & $T^{\mathrm{total}}_{\mathrm{opt}}$ [ms]
      & $T^{\mathrm{total}}_{\mathrm{replan}}$ [ms]
      & $T_{\mathrm{trav}}$ [s]
      & $L_{\mathrm{path}}$ [m]
      & $S_{\mathrm{jerk}}$ [m/s$^{2}$]
      & $\rho_{\mathrm{vel}}$ [\%]
      & $\rho_{\mathrm{acc}}$ [\%]
      & $\rho_{\mathrm{jerk}}$ [\%]
      \\
      \midrule

      \multirow{5}{*}{Easy} & EGO-Swarm2 & Soft & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & \multirow{2}{*}{SUPER} & \multirow{2}{*}{Soft} & $L_2$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & & & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & FASTER & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & DYNUS & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\

      \midrule

      \multirow{5}{*}{Medium} & EGO-Swarm2 & Soft & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & \multirow{2}{*}{SUPER} & \multirow{2}{*}{Soft} & $L_2$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & & & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & FASTER & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & DYNUS & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\

      \midrule

      \multirow{5}{*}{Hard} & EGO-Swarm2 & Soft & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & \multirow{2}{*}{SUPER} & \multirow{2}{*}{Soft} & $L_2$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & & & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & FASTER & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\
       & DYNUS & Hard & $L_\infty$ & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\

      \bottomrule
    \end{tabular}
  }
  \vspace{-1.0em}
\end{table*}
"""


def load_obstacles(obstacle_csv_path):
    """
    Load static obstacle cylinders from a CSV file.

    Expected columns: id,shape,x,y,z,radius,width,depth,height,yaw
    Returns a list of dicts with keys: x, y, z, radius, height.
    """
    obstacles = []
    with open(obstacle_csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["shape"].strip() != "cylinder":
                continue
            obstacles.append({
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "radius": float(row["radius"]),
                "height": float(row["height"]),
            })
    return obstacles


def check_collisions(positions, obstacles, robot_r=ROBOT_RADIUS):
    """
    Check a trajectory (list of (x,y,z) tuples) against cylinder obstacles.

    Uses point-mass model: collision occurs when the point is inside the
    cylinder (horiz_dist < obstacle_radius) AND within the cylinder's
    vertical extent.

    Returns (num_collisions, num_samples, min_clearance).
      - num_collisions: number of position samples that collide
      - num_samples: total position samples checked
      - min_clearance: smallest horizontal clearance across all samples/obstacles
    """
    num_collisions = 0
    min_clearance = float("inf")

    for pos in positions:
        px, py, pz = pos
        for obs in obstacles:
            # Vertical check: cylinder spans [z - h/2, z + h/2]
            half_h = obs["height"] / 2.0
            if pz < obs["z"] - half_h or pz > obs["z"] + half_h:
                continue
            # Horizontal distance to cylinder axis
            dx = px - obs["x"]
            dy = py - obs["y"]
            horiz_dist = math.sqrt(dx * dx + dy * dy)
            clearance = horiz_dist - obs["radius"]
            min_clearance = min(min_clearance, clearance)
            if horiz_dist < obs["radius"]:
                num_collisions += 1
                break  # one collision per sample is enough

    if min_clearance == float("inf"):
        min_clearance = 0.0

    return num_collisions, len(positions), min_clearance


def compute_collision_stats(data_dir, obstacles, v_max, a_max, j_max, goal=None):
    """
    Check all bag files in data_dir for collisions against static obstacles.

    Returns dict with collision_count, total_samples, min_clearance, trials_with_collision,
    and collision_per_trial (dict mapping sim_num -> bool).
    """
    bag_files = glob.glob(os.path.join(data_dir, "super_num_*.bag"))
    if not bag_files:
        return None

    bag_files = sorted(bag_files, key=lambda f: extract_number(os.path.basename(f)))

    total_collisions = 0
    total_samples = 0
    overall_min_clearance = float("inf")
    trials_with_collision = 0
    collision_per_trial = {}

    for bag_file in bag_files:
        sim_num = extract_number(os.path.basename(bag_file))
        result = process_bag(bag_file, tol=1.0,
                             v_constraint=v_max,
                             a_constraint=a_max,
                             j_constraint=j_max,
                             goal=goal)
        if result is None or "positions" not in result:
            continue

        n_col, n_samp, min_cl = check_collisions(result["positions"], obstacles)
        total_collisions += n_col
        total_samples += n_samp
        overall_min_clearance = min(overall_min_clearance, min_cl)
        collision_per_trial[sim_num] = (n_col > 0)
        if n_col > 0:
            trials_with_collision += 1
            print(f"    {os.path.basename(bag_file)}: {n_col} collision samples, "
                  f"min clearance = {min_cl:.3f} m")

    if overall_min_clearance == float("inf"):
        overall_min_clearance = 0.0

    return {
        "collision_count": total_collisions,
        "total_samples": total_samples,
        "min_clearance": overall_min_clearance,
        "trials_with_collision": trials_with_collision,
        "total_trials": len(bag_files),
        "collision_per_trial": collision_per_trial,
    }


# Timeout threshold [s] for success: trials with travel_time > this are failures
TIMEOUT_THRESHOLD = 50.0


def compute_success_rate(data_dir, collision_per_trial=None, travel_times_per_trial=None):
    """Compute success rate from goal_reached_status_super.csv.

    A trial is successful only if:
      1. goal was reached (status == "reached")
      2. no collisions detected (if collision data is provided)
      3. travel_time <= TIMEOUT_THRESHOLD (if travel time data is provided)

    collision_per_trial: dict mapping sim_num (int) -> bool (True if collision)
    travel_times_per_trial: dict mapping sim_num (int) -> float (travel time in seconds)
    """
    csv_path = os.path.join(data_dir, "goal_reached_status_super.csv")
    if not os.path.exists(csv_path):
        print(f"  Warning: {csv_path} not found, assuming 0% success")
        return 0.0, 0, 0

    total = 0
    succeeded = 0
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total += 1
            sim_num = int(row["sim_num"])
            if row["status"].strip() != "reached":
                continue
            # Check collision-free
            if collision_per_trial is not None and collision_per_trial.get(sim_num, False):
                continue
            # Check travel time within timeout
            if travel_times_per_trial is not None:
                tt = travel_times_per_trial.get(sim_num, None)
                if tt is not None and tt > TIMEOUT_THRESHOLD:
                    continue
            succeeded += 1

    rate = (succeeded / total * 100.0) if total > 0 else 0.0
    return rate, succeeded, total


def compute_bag_statistics(data_dir, v_max, a_max, j_max, goal=None):
    """Analyze all bag files and return averaged statistics.

    Also returns travel_times_per_trial: dict mapping sim_num -> travel_time.
    """
    bag_files = glob.glob(os.path.join(data_dir, "super_num_*.bag"))
    if not bag_files:
        print("  No bag files found in", data_dir)
        return None

    bag_files = sorted(bag_files, key=lambda f: extract_number(os.path.basename(f)))
    print(f"  Found {len(bag_files)} bag file(s)")

    travel_times = []
    path_lengths = []
    smoothness_vals = []
    total_vel_viols = 0
    total_acc_viols = 0
    total_jerk_viols = 0
    total_cmds = 0
    travel_times_per_trial = {}

    for bag_file in bag_files:
        sim_num = extract_number(os.path.basename(bag_file))
        result = process_bag(bag_file, tol=1.0,
                             v_constraint=v_max,
                             a_constraint=a_max,
                             j_constraint=j_max,
                             goal=goal)
        if result is None:
            continue

        travel_times.append(result["travel_time"])
        path_lengths.append(result["path_length"])
        smoothness_vals.append(result["smoothness"])
        total_vel_viols += result["vel_violations"]
        total_acc_viols += result["acc_violations"]
        total_jerk_viols += result["jerk_violations"]
        total_cmds += result["total_pos_cmds"]
        travel_times_per_trial[sim_num] = result["travel_time"]

    if not travel_times:
        return None

    return {
        "travel_time": float(np.mean(travel_times)),
        "path_length": float(np.mean(path_lengths)),
        "smoothness": float(np.mean(smoothness_vals)),
        "vel_violation_pct": (total_vel_viols / total_cmds * 100.0) if total_cmds > 0 else 0.0,
        "acc_violation_pct": (total_acc_viols / total_cmds * 100.0) if total_cmds > 0 else 0.0,
        "jerk_violation_pct": (total_jerk_viols / total_cmds * 100.0) if total_cmds > 0 else 0.0,
        "travel_times_per_trial": travel_times_per_trial,
    }


def compute_computation_times(data_dir):
    """Compute average computation times from time_consuming_num_*.csv files"""
    pattern = os.path.join(data_dir, "time_consuming_num_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print("  No computation time CSV files found")
        return None

    track_cols = {
        "GENERATE_BACK_TRAJ", "GENERATE_EXP_TRAJ", "TOTAL_REPLAN",
        "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
        "EPX_TRAJ_FRONTEND", "EPX_TRAJ_GLOBAL",
        "EXP_TRAJ_INIT", "EXP_TRAJ_OPT",
    }

    global_stats = {col: RunningStats() for col in track_cols | {"EXP_SUCCESS"}}

    for fp in files:
        compute_file_stats(fp, track_cols, global_stats)

    result = {}
    for col in global_stats:
        rs = global_stats[col]
        if rs.n > 0:
            result[col] = rs.avg

    print(f"  Loaded {len(files)} computation time file(s)")
    return result


def format_violation(val):
    """Format violation percentage with appropriate precision"""
    if val == 0.0:
        return "0.0"
    elif val < 0.1:
        return f"{val:.2f}"
    else:
        return f"{val:.1f}"


def format_super_row(success_rate, bag_stats, comp_stats):
    """Build a SUPER data row string (the cells after '& SUPER & {Soft} &')."""
    exp_time = comp_stats.get("GENERATE_EXP_TRAJ", 0.0) if comp_stats else 0.0
    back_time = comp_stats.get("GENERATE_BACK_TRAJ", 0.0) if comp_stats else 0.0
    total_replan = comp_stats.get("TOTAL_REPLAN", 0.0) if comp_stats else 0.0

    travel_time = bag_stats["travel_time"] if bag_stats else 0.0
    path_length = bag_stats["path_length"] if bag_stats else 0.0
    smoothness = bag_stats["smoothness"] if bag_stats else 0.0
    vel_viol = bag_stats["vel_violation_pct"] if bag_stats else 0.0
    acc_viol = bag_stats["acc_violation_pct"] if bag_stats else 0.0
    jerk_viol = bag_stats["jerk_violation_pct"] if bag_stats else 0.0

    return (
        f"{{{success_rate:.1f}}} "
        f"& {exp_time:.1f} | {back_time:.1f} "
        f"& {{{total_replan:.1f}}} "
        f"& {{{travel_time:.1f}}} & {{{path_length:.1f}}} & {{{smoothness:.1f}}} "
        f"& {{{format_violation(vel_viol)}}} & {{{format_violation(acc_viol)}}} "
        f"& {{{format_violation(jerk_viol)}}} \\\\"
    )


def _ensure_table_exists(latex_path):
    """Ensure the LaTeX table file exists, creating it from template if needed."""
    if not os.path.exists(latex_path):
        print(f"  Creating table at {latex_path}")
        os.makedirs(os.path.dirname(latex_path), exist_ok=True)
        with open(latex_path, 'w') as f:
            f.write(DEFAULT_TABLE_TEMPLATE)


def update_super_row_for_env(success_rate, bag_stats, comp_stats, latex_path, env_label):
    """
    Update the SUPER row within a specific environment section of the table.

    Finds the section headed by \\multirow{...}{env_label}, then replaces the
    SUPER row within that section.

    env_label: one of "Easy", "Medium", "Hard"
    """
    _ensure_table_exists(latex_path)

    data_cells = format_super_row(success_rate, bag_stats, comp_stats)
    super_row = f"       & SUPER & {{Soft}} & {data_cells}"

    with open(latex_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    current_env = None
    replaced = False

    for line in lines:
        stripped = line.strip()

        # Track which environment section we're in via \multirow{...}{Label}
        if "\\multirow" in stripped:
            for label in ENV_TO_LABEL.values():
                if "{" + label + "}" in stripped:
                    current_env = label
                    break

        # Reset env tracking at section breaks
        if stripped == "\\midrule" or stripped == "\\bottomrule":
            current_env = None

        # Match SUPER row: starts with "& SUPER" (indented, within an env section)
        if (current_env == env_label and
                "SUPER" in stripped and "Soft" in stripped and "&" in stripped):
            new_lines.append(super_row + "\n")
            replaced = True
        else:
            new_lines.append(line)

    if not replaced:
        print(f"  Warning: SUPER row not found in {env_label} section, could not update")
        return False

    with open(latex_path, 'w') as f:
        f.writelines(new_lines)

    return True


def _match_super_planner_row(stripped, planner_name):
    """
    Check if a stripped line matches the SUPER row for a given planner variant.

    Table format uses \\multirow{2}{*}{SUPER} and \\multirow{2}{*}{Soft} on
    the L2 row, and the L-inf row is a continuation starting with '& & &':
      L2:   & \\multirow{2}{*}{SUPER} & \\multirow{2}{*}{Soft} & $L_2$ & ...
      Linf: & & & $L_\\infty$ & ...
    """
    if planner_name == "super_l2":
        return "SUPER" in stripped and "$L_2$" in stripped
    elif planner_name == "super_linf":
        # The L-inf row is the multirow continuation: starts with "& & &"
        # (empty Algorithm and Constr-type cells) and contains "$L_\\infty$".
        # Other rows have their algorithm name after the first "&", so they
        # never start with "& & &".
        return stripped.startswith("& & &") and "$L_\\infty$" in stripped
    return False


def _build_super_row(planner_name, data_cells):
    """Build the full replacement row for a SUPER planner variant."""
    if planner_name == "super_l2":
        return f"       & \\multirow{{2}}{{*}}{{SUPER}} & \\multirow{{2}}{{*}}{{Soft}} & $L_2$ & {data_cells}"
    elif planner_name == "super_linf":
        return f"       & & & $L_\\infty$ & {data_cells}"
    return None


def update_super_row_for_planner_env(success_rate, bag_stats, comp_stats,
                                     latex_path, env_label, planner_name):
    """
    Update a specific SUPER row identified by both environment section and constraint label.

    Matches the row by env_label (e.g. "Easy") AND the planner variant
    (super_l2 or super_linf), handling the multirow SUPER format.
    """
    constr_label = PLANNER_TO_CONSTR_LABEL.get(planner_name)
    if constr_label is None:
        print(f"  Warning: no constraint label mapping for planner '{planner_name}'")
        return False

    _ensure_table_exists(latex_path)

    data_cells = format_super_row(success_rate, bag_stats, comp_stats)
    super_row = _build_super_row(planner_name, data_cells)
    if super_row is None:
        print(f"  Warning: unknown planner '{planner_name}'")
        return False

    with open(latex_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    current_env = None
    replaced = False

    for line in lines:
        stripped = line.strip()

        # Track which environment section we're in via \multirow{...}{Label}
        # Only match the env-level multirow (e.g. \multirow{5}{*}{Easy}), not
        # the SUPER-level \multirow{2}{*}{SUPER}
        if "\\multirow" in stripped and "SUPER" not in stripped:
            for label in ENV_TO_LABEL.values():
                if "{" + label + "}" in stripped:
                    current_env = label
                    break

        # Reset env tracking at section breaks
        if stripped == "\\midrule" or stripped == "\\bottomrule":
            current_env = None

        # Match SUPER row for this planner in the right env section
        if (current_env == env_label and
                _match_super_planner_row(stripped, planner_name)):
            new_lines.append(super_row + "\n")
            replaced = True
        else:
            new_lines.append(line)

    if not replaced:
        print(f"  Warning: SUPER {constr_label} row not found in {env_label} section")
        return False

    with open(latex_path, 'w') as f:
        f.writelines(new_lines)

    return True


def update_super_row(success_rate, bag_stats, comp_stats, latex_path):
    """Update the first SUPER row found in the table (legacy single-env mode)."""
    _ensure_table_exists(latex_path)

    data_cells = format_super_row(success_rate, bag_stats, comp_stats)

    with open(latex_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    replaced = False
    for line in lines:
        stripped = line.strip()
        if not replaced and "SUPER" in stripped and "Soft" in stripped and "&" in stripped:
            # Detect whether this is a per-env row (starts with &) or old-style row
            if stripped.startswith("& SUPER") or stripped.startswith("&SUPER"):
                new_lines.append(f"       & SUPER & {{Soft}} & {data_cells}\n")
            else:
                new_lines.append(f"      SUPER        & {{Soft}} & {data_cells}\n")
            replaced = True
        else:
            new_lines.append(line)

    if not replaced:
        print("  Warning: SUPER row not found in table, could not update")
        return False

    with open(latex_path, 'w') as f:
        f.writelines(new_lines)

    return True


def print_summary(success_rate, reached, total, bag_stats, comp_stats,
                  collision_stats=None, label=""):
    """Print a formatted summary of the results"""
    header = "SUPER STATIC BENCHMARK RESULTS"
    if label:
        header += f" - {label}"

    print("\n" + "=" * 70)
    print(header)
    print("=" * 70)

    print(f"\n  Success Rate: {success_rate:.1f}% ({reached}/{total})")

    if collision_stats:
        col = collision_stats
        print(f"\n  Collision Check:")
        print(f"    Trials with collision: {col['trials_with_collision']}/{col['total_trials']}")
        print(f"    Collision samples:     {col['collision_count']}/{col['total_samples']}")
        print(f"    Min clearance:         {col['min_clearance']:.3f} m")

    if comp_stats:
        exp_time = comp_stats.get("GENERATE_EXP_TRAJ", 0.0)
        back_time = comp_stats.get("GENERATE_BACK_TRAJ", 0.0)
        total_replan = comp_stats.get("TOTAL_REPLAN", 0.0)
        print(f"\n  Computation Time:")
        print(f"    T_opt (Exploratory | Safe): {exp_time:.1f} | {back_time:.1f} ms")
        print(f"    T_replan (Total):           {total_replan:.1f} ms")

    if bag_stats:
        print(f"\n  Performance:")
        print(f"    Travel time:  {bag_stats['travel_time']:.1f} s")
        print(f"    Path length:  {bag_stats['path_length']:.1f} m")
        print(f"    Smoothness:   {bag_stats['smoothness']:.1f} m/s^2")
        print(f"\n  Constraint Violations:")
        print(f"    Velocity:     {bag_stats['vel_violation_pct']:.2f}%")
        print(f"    Acceleration: {bag_stats['acc_violation_pct']:.2f}%")
        print(f"    Jerk:         {bag_stats['jerk_violation_pct']:.2f}%")

    print("\n" + "=" * 70)


def analyze_single_env(data_dir, v_max, a_max, j_max, label="", obstacles=None, goal=None):
    """
    Analyze a single environment directory.

    Returns (success_rate, reached, total, bag_stats, comp_stats, collision_stats).
    collision_stats is None if no obstacles are provided.
    """
    if label:
        print(f"\n--- Analyzing: {label} ---")
        print(f"  Directory: {data_dir}")

    # 1. Bag analysis (needed for travel times before success rate)
    print("Analyzing bag files...")
    bag_stats = compute_bag_statistics(data_dir, v_max, a_max, j_max, goal=goal)
    if bag_stats:
        print(f"  Travel time: {bag_stats['travel_time']:.1f} s")
        print(f"  Path length: {bag_stats['path_length']:.1f} m")
        print(f"  Smoothness:  {bag_stats['smoothness']:.1f} m/s^2\n")

    # 2. Collision check against static obstacles
    collision_stats = None
    if obstacles:
        print(f"Checking collisions against {len(obstacles)} obstacles...")
        collision_stats = compute_collision_stats(data_dir, obstacles, v_max, a_max, j_max, goal=goal)
        if collision_stats:
            if collision_stats["trials_with_collision"] == 0:
                print("  No collisions detected\n")
            else:
                print(f"  {collision_stats['trials_with_collision']} trial(s) with collisions\n")

    # 3. Success rate (requires collision and travel time data)
    collision_per_trial = collision_stats["collision_per_trial"] if collision_stats else None
    travel_times_per_trial = bag_stats["travel_times_per_trial"] if bag_stats else None
    print("Computing success rate...")
    success_rate, reached, total = compute_success_rate(
        data_dir,
        collision_per_trial=collision_per_trial,
        travel_times_per_trial=travel_times_per_trial,
    )
    print(f"  Success rate: {success_rate:.1f}% ({reached}/{total})\n")

    # 4. Computation times
    print("Loading computation times...")
    comp_stats = compute_computation_times(data_dir)
    if comp_stats:
        print(f"  Exp traj:     {comp_stats.get('GENERATE_EXP_TRAJ', 0):.1f} ms")
        print(f"  Back traj:    {comp_stats.get('GENERATE_BACK_TRAJ', 0):.1f} ms")
        print(f"  Total replan: {comp_stats.get('TOTAL_REPLAN', 0):.1f} ms\n")

    return success_rate, reached, total, bag_stats, comp_stats, collision_stats


def aggregate_results(env_results):
    """
    Aggregate results across multiple environments.

    env_results: list of (success_rate, reached, total, bag_stats, comp_stats, collision_stats) tuples
    Returns aggregated (success_rate, reached, total, bag_stats, comp_stats, collision_stats).
    """
    total_reached = sum(r[1] for r in env_results)
    total_trials = sum(r[2] for r in env_results)
    agg_success = (total_reached / total_trials * 100.0) if total_trials > 0 else 0.0

    # Aggregate bag stats by averaging across environments
    bag_keys = ["travel_time", "path_length", "smoothness",
                "vel_violation_pct", "acc_violation_pct", "jerk_violation_pct"]
    valid_bag = [r[3] for r in env_results if r[3] is not None]
    if valid_bag:
        agg_bag = {}
        for key in bag_keys:
            agg_bag[key] = float(np.mean([b[key] for b in valid_bag]))
    else:
        agg_bag = None

    # Aggregate computation stats by averaging across environments
    valid_comp = [r[4] for r in env_results if r[4] is not None]
    if valid_comp:
        all_keys = set()
        for c in valid_comp:
            all_keys.update(c.keys())
        agg_comp = {}
        for key in all_keys:
            vals = [c[key] for c in valid_comp if key in c]
            if vals:
                agg_comp[key] = float(np.mean(vals))
    else:
        agg_comp = None

    # Aggregate collision stats by summing counts
    valid_col = [r[5] for r in env_results if r[5] is not None]
    if valid_col:
        agg_col = {
            "collision_count": sum(c["collision_count"] for c in valid_col),
            "total_samples": sum(c["total_samples"] for c in valid_col),
            "min_clearance": min(c["min_clearance"] for c in valid_col),
            "trials_with_collision": sum(c["trials_with_collision"] for c in valid_col),
            "total_trials": sum(c["total_trials"] for c in valid_col),
        }
    else:
        agg_col = None

    return agg_success, total_reached, total_trials, agg_bag, agg_comp, agg_col


def main():
    parser = argparse.ArgumentParser(
        description="Analyze SUPER static benchmark data and update LaTeX table",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--data-dir", type=str, required=True,
                        help="Path to benchmark data directory")
    parser.add_argument("--latex-path", type=str, default=DEFAULT_LATEX_PATH,
                        help=f"Path to LaTeX table (default: {DEFAULT_LATEX_PATH})")
    parser.add_argument("--v-max", type=float, default=10.0,
                        help="Velocity constraint for violation check [m/s] (default: 10.0)")
    parser.add_argument("--a-max", type=float, default=20.0,
                        help="Acceleration constraint for violation check [m/s^2] (default: 20.0)")
    parser.add_argument("--j-max", type=float, default=30.0,
                        help="Jerk constraint for violation check [m/s^3] (default: 30.0)")
    parser.add_argument("--envs", type=str, default=None,
                        help="Comma-separated environment names (e.g. easy_forest,medium_forest,hard_forest)")
    parser.add_argument("--obstacle-dir", type=str, default=None,
                        help="Directory containing <env>_obstacle_parameters.csv files for collision checking")
    parser.add_argument("--planners", type=str, default=None,
                        help="Comma-separated planner names (e.g. super_l2,super_linf). "
                             "When specified, data is expected under <data-dir>/<planner>/<env>/")
    parser.add_argument("--goal", type=str, default=None,
                        help="Goal coordinates as x,y,z (e.g. 105.0,0.0,2.0). "
                             "If not specified, defaults to (305,0,3).")

    args = parser.parse_args()

    print("=" * 70)
    print("SUPER STATIC BENCHMARK ANALYZER")
    print("=" * 70)
    print(f"\n  Data directory: {args.data_dir}")
    print(f"  LaTeX output:   {args.latex_path}")
    print(f"  Constraints:    v={args.v_max}, a={args.a_max}, j={args.j_max}")
    if args.obstacle_dir:
        print(f"  Obstacle dir:   {args.obstacle_dir}")
    if args.planners:
        print(f"  Planners:       {args.planners}")

    # Parse goal coordinates if specified
    goal = None
    if args.goal:
        goal = tuple(map(float, args.goal.split(",")))
        print(f"  Goal:           {goal}")

    # Parse planner names if specified
    planner_names = None
    if args.planners:
        planner_names = [p.strip() for p in args.planners.split(",")]

    if args.envs:
        # Multi-environment mode
        env_names = [e.strip() for e in args.envs.split(",")]
        print(f"  Environments:   {env_names}\n")

        if planner_names:
            # Multi-planner + multi-environment mode
            all_ok = True
            for planner_name in planner_names:
                print(f"\n{'#'*70}")
                print(f"  PLANNER: {planner_name}")
                print(f"{'#'*70}")

                env_results = []
                env_results_with_name = []

                for env_name in env_names:
                    env_dir = os.path.join(args.data_dir, planner_name, env_name)
                    if not os.path.isdir(env_dir):
                        print(f"  Warning: {env_dir} does not exist, skipping {planner_name}/{env_name}")
                        continue

                    obstacles = None
                    if args.obstacle_dir:
                        obs_csv = os.path.join(args.obstacle_dir,
                                               f"{env_name}_obstacle_parameters.csv")
                        if os.path.exists(obs_csv):
                            obstacles = load_obstacles(obs_csv)
                            print(f"  Loaded {len(obstacles)} obstacles from {obs_csv}")
                        else:
                            print(f"  Warning: {obs_csv} not found, skipping collision check")

                    result = analyze_single_env(env_dir, args.v_max, args.a_max, args.j_max,
                                                label=f"{planner_name}/{env_name}",
                                                obstacles=obstacles, goal=goal)
                    env_results.append(result)
                    env_results_with_name.append((env_name, result))

                    print_summary(*result, label=f"{planner_name}/{env_name}")

                if not env_results:
                    print(f"ERROR: No environment data found for planner {planner_name}")
                    continue

                # Aggregate across environments for this planner
                agg_result = aggregate_results(env_results)
                print_summary(*agg_result, label=f"{planner_name} AGGREGATE")

                # Update LaTeX table with planner-specific rows
                for env_name, (success_rate, reached, total, bag_stats, comp_stats, _col) in env_results_with_name:
                    env_label = ENV_TO_LABEL.get(env_name)
                    if env_label is None:
                        print(f"  Warning: no table label mapping for '{env_name}', skipping LaTeX update")
                        continue
                    constr_label = PLANNER_TO_CONSTR_LABEL.get(planner_name, planner_name)
                    print(f"\nUpdating SUPER {constr_label} row for {env_label} in {args.latex_path}...")
                    if update_super_row_for_planner_env(success_rate, bag_stats, comp_stats,
                                                        args.latex_path, env_label, planner_name):
                        print(f"  Successfully updated SUPER {constr_label} row in {env_label} section")
                    else:
                        print(f"  Failed to update SUPER {constr_label} row for {env_label}")
                        all_ok = False

            if not all_ok:
                sys.exit(1)

        else:
            # Single-planner + multi-environment mode (original behavior)
            env_results = []
            env_results_with_name = []
            for env_name in env_names:
                env_dir = os.path.join(args.data_dir, env_name)
                if not os.path.isdir(env_dir):
                    print(f"  Warning: {env_dir} does not exist, skipping {env_name}")
                    continue

                # Load obstacles for this environment if obstacle dir is provided
                obstacles = None
                if args.obstacle_dir:
                    obs_csv = os.path.join(args.obstacle_dir,
                                           f"{env_name}_obstacle_parameters.csv")
                    if os.path.exists(obs_csv):
                        obstacles = load_obstacles(obs_csv)
                        print(f"  Loaded {len(obstacles)} obstacles from {obs_csv}")
                    else:
                        print(f"  Warning: {obs_csv} not found, skipping collision check")

                result = analyze_single_env(env_dir, args.v_max, args.a_max, args.j_max,
                                            label=env_name, obstacles=obstacles, goal=goal)
                env_results.append(result)
                env_results_with_name.append((env_name, result))

                # Print per-env summary
                print_summary(*result, label=env_name)

            if not env_results:
                print("ERROR: No environment data found")
                sys.exit(1)

            # Aggregate across all environments
            agg_result = aggregate_results(env_results)
            print_summary(*agg_result, label="AGGREGATE")

            # Update LaTeX table with per-environment SUPER rows
            all_ok = True
            for env_name, (success_rate, reached, total, bag_stats, comp_stats, _col) in env_results_with_name:
                env_label = ENV_TO_LABEL.get(env_name)
                if env_label is None:
                    print(f"  Warning: no table label mapping for '{env_name}', skipping LaTeX update")
                    continue
                print(f"\nUpdating SUPER row for {env_label} in {args.latex_path}...")
                if update_super_row_for_env(success_rate, bag_stats, comp_stats,
                                            args.latex_path, env_label):
                    print(f"  Successfully updated SUPER row in {env_label} section")
                else:
                    print(f"  Failed to update SUPER row for {env_label}")
                    all_ok = False

            if not all_ok:
                sys.exit(1)

    else:
        # Single-environment mode (original behavior)
        print()

        # Load obstacles if provided
        obstacles = None
        if args.obstacle_dir:
            # In single-env mode, look for any obstacle CSV in the dir
            obs_files = glob.glob(os.path.join(args.obstacle_dir, "*_obstacle_parameters.csv"))
            if obs_files:
                obstacles = load_obstacles(obs_files[0])
                print(f"  Loaded {len(obstacles)} obstacles from {obs_files[0]}")

        result = analyze_single_env(
            args.data_dir, args.v_max, args.a_max, args.j_max, obstacles=obstacles, goal=goal
        )
        success_rate, reached, total, bag_stats, comp_stats, collision_stats = result

        # Print summary
        print_summary(success_rate, reached, total, bag_stats, comp_stats,
                      collision_stats=collision_stats)

        # Update LaTeX table
        print(f"\nUpdating SUPER row in {args.latex_path}...")
        if update_super_row(success_rate, bag_stats, comp_stats, args.latex_path):
            print(f"  Successfully updated SUPER row in {args.latex_path}")
        else:
            print(f"  Failed to update SUPER row")
            sys.exit(1)

    print("\nDone.")


if __name__ == "__main__":
    main()
