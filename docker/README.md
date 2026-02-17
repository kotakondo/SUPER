# SUPER Docker Benchmarking

Instructions for running SUPER benchmarks inside Docker. Two benchmark types are available:

1. **Static Forest Benchmark** — full closed-loop simulation (planner + drone sim + rosbag) across easy/medium/hard forests
2. **Standardized Benchmark** — offline trajectory optimization on pre-generated safety corridors (`.mysco2` files from Dynus)

## Prerequisites

- Docker with NVIDIA GPU support (`nvidia-docker2`)
- Paper writing directory at `~/paper_writing/DYNUS_v3/tables/` (for LaTeX table output)

Additional prerequisites per benchmark type:

**Static Forest Benchmark:**
- Dynus world files at `~/code/dynus_ws/src/dynus/worlds/` (`easy_forest.world`, `medium_forest.world`, `hard_forest.world`)
- Dynus obstacle parameters at `~/code/dynus_ws/src/dynus/benchmark_data/static/` (used for collision checking)

**Standardized Benchmark:**
- Pre-generated `.mysco2` safety corridor files at `~/code/dynus_ws/src/dynus/data/`
- `generate_latex_table.py` at `~/code/dynus_ws/src/dynus/benchmarking/` (for LaTeX table generation)

## 1. Build the Docker Image

```bash
cd docker/
make build
```

To rebuild from scratch (no cache):

```bash
make build-no-cache
```

---

# Static Forest Benchmark

## 2. Run the Static Forest Benchmark

This runs both planner variants (L2 and L-inf constraint norms) across all three forest environments (easy, medium, hard).

```bash
cd docker/
make run-static-benchmark NUM_TRIALS=10
```

### What happens

1. **World-to-PCD conversion**: Converts Gazebo `.world` files to `.pcd` point clouds for the drone simulator
2. **Simulation**: For each planner variant (`super_l2`, `super_linf`) and each environment (`easy_forest`, `medium_forest`, `hard_forest`), runs `NUM_TRIALS` trials:
   - Launches the planner + drone simulator
   - Records a rosbag
   - Publishes goal at `(105.0, 0.0, 2.0)`
   - Monitors until goal is reached or timeout (50s)
3. **Analysis**: Processes all rosbags and computation time CSVs, then updates the LaTeX table

### Parameters

| Variable | Default | Description |
|----------|---------|-------------|
| `NUM_TRIALS` | `10` | Number of trials per planner/environment combination |
| `SKIP_BUILD` | `1` | Skip `catkin_make` inside Docker. Set to `0` if code has changed |

### Planner configs

- **L2**: `super_planner/config/static_forest_l2.yaml` — uses `constraint_norm_type: 0`
- **L-inf**: `super_planner/config/static_forest.yaml` — uses `constraint_norm_type: 1`

To rebuild code changes before running:

```bash
make run-static-benchmark NUM_TRIALS=10 SKIP_BUILD=0
```

## 3. Data Organization

Data is saved to `SUPER/data/<TIMESTAMP>/` with this structure:

```
data/<TIMESTAMP>/
  super_l2/
    easy_forest/
      super_num_0.bag
      super_num_1.bag
      ...
      time_consuming_num_0.csv
      time_consuming_num_1.csv
      ...
      goal_reached_status_super.csv
    medium_forest/
      ...
    hard_forest/
      ...
  super_linf/
    easy_forest/
      ...
    medium_forest/
      ...
    hard_forest/
      ...
```

### Key files per trial

| File | Description |
|------|-------------|
| `super_num_<N>.bag` | Full rosbag recording of the trial |
| `time_consuming_num_<N>.csv` | Per-replan computation times from the planner |
| `goal_reached_status_super.csv` | Summary of reach/timeout status for all trials |

## 4. Re-analyze Existing Data

To re-run only the analysis step on previously collected data (no simulation):

```bash
cd docker/
make analyze-static-benchmark DATA_DIR=<TIMESTAMP>
```

For example:

```bash
make analyze-static-benchmark DATA_DIR=20260217_143000
```

To list available benchmark runs:

```bash
ls -d ../data/*/
```

## 5. LaTeX Table Output

The analysis script updates the SUPER rows in:

```
~/paper_writing/DYNUS_v3/tables/static_benchmark.tex
```

The table has two SUPER rows per environment:
- **SUPER / Soft / L2**: Results using L2 constraint norm
- **SUPER / Soft / L-inf**: Results using L-inf constraint norm

### Metrics in the table

| Column | Description |
|--------|-------------|
| R_succ | Success rate (%) — goal reached, collision-free, within timeout |
| T_opt | Total optimization computation time (ms) — reported as `Exploratory \| Safe` |
| T_replan | Total replanning time (ms) |
| T_trav | Travel time (s) |
| L_path | Path length (m) |
| S_jerk | Jerk smoothness (m/s^2) |
| rho_vel | Velocity constraint violation (%) |
| rho_acc | Acceleration constraint violation (%) |
| rho_jerk | Jerk constraint violation (%) |

### Constraint limits used for violation checking

- v_max: 5.0 m/s
- a_max: 20.0 m/s^2
- j_max: 100.0 m/s^3

---

# Standardized Benchmark

The standardized benchmark runs SUPER's trajectory optimizer offline on pre-generated safety corridor files (`.mysco2` format) from Dynus. This does **not** run a full simulation — it reads each corridor, optimizes a trajectory, and writes results to a CSV.

## 6. Run the Standardized Benchmark

```bash
cd docker/
make run-standardized-benchmark
```

### What happens

1. Reads all `.mysco2` files from `~/code/dynus_ws/src/dynus/data/` (101 corridor scenarios)
2. For each file, loads the safety corridor polytopes, start/goal, and guide trajectory
3. Runs the SUPER `ExpTrajOpt` optimizer with config `super_planner/config/benchmark.yaml`
4. Analyzes each trajectory for constraint violations (corridor, velocity, acceleration, jerk)
5. Writes results to a CSV

### Config

The optimizer config is at `super_planner/config/benchmark.yaml`:
- `constraint_norm_type: 1` (L-inf)
- `max_vel: 1.0`, `max_acc: 2.0`, `max_jerk: 3.0`

### Output

The CSV is written to:

```
~/code/dynus_ws/src/dynus/benchmark_data/default/super_benchmark.csv
```

Each row contains per-scenario results including:
- `success` — whether optimization succeeded
- `per_opt_runtime_ms` / `total_opt_runtime_ms` — computation time
- `total_traj_time_sec` — trajectory duration
- `traj_length_m` — path length
- `jerk_smoothness_l1` / `jerk_rms` — smoothness metrics
- `corridor_violated`, `v_violated`, `a_violated`, `j_violated` — constraint violation flags
- `v_max_observed`, `a_max_observed`, `j_max_observed` — peak observed values

To rebuild code changes before running:

```bash
make run-standardized-benchmark SKIP_BUILD=0
```

## 7. Analyze Standardized Benchmark Results

After generating the CSV, update the LaTeX table:

```bash
cd docker/
make analyze-standardized-benchmark
```

This runs `generate_latex_table.py` (from the Dynus repo) on the host machine and updates:

```
~/paper_writing/DYNUS_v3/tables/standardized_benchmark.tex
```

**Note:** This analysis step runs on the host (not inside Docker) and requires `pandas` and `numpy` installed in your local Python environment.

## 8. End-to-End Workflow (Standardized)

```bash
cd docker/

# Step 1: Generate trajectories and write CSV
make run-standardized-benchmark

# Step 2: Analyze CSV and update LaTeX table
make analyze-standardized-benchmark
```

---

# Utilities

## 9. Interactive Shell (Debugging)

To get an interactive shell inside the container with all volumes mounted:

```bash
cd docker/
make shell
```

This mounts the SUPER source code read-write so you can edit and rebuild inside the container.
