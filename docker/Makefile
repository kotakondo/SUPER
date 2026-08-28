.PHONY: build build-no-cache run run-benchmark analyze-benchmark shell \
       run-standardized-benchmark analyze-standardized-benchmark \
       run-static-benchmark analyze-static-benchmark \
       record-video record-video-headless replay-bag

# Default number of benchmark trials
NUM_TRIALS ?= 10

# Set SKIP_BUILD=1 to skip catkin_make (use if code hasn't changed since last build)
SKIP_BUILD ?= 1

# Host SUPER root directory (parent of docker/)
SUPER_ROOT := $(shell cd .. && pwd)

# Timestamp for benchmark data directory (generated at make invocation time)
TIMESTAMP := $(shell date +%Y%m%d_%H%M%S)

# Common Docker flags
DOCKER_FLAGS := \
	--gpus all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--privileged \
	-e DISPLAY=${DISPLAY} \
	--network=host \
	--env="QT_X11_NO_MITSHM=1"

# Common setup commands inside the container
ifeq ($(SKIP_BUILD),1)
SETUP_CMD := \
	cd /home/kota/super_ws && \
	export DISABLE_ROS1_EOL_WARNINGS=1 && \
	source /opt/ros/noetic/setup.bash && \
	bash src/SUPER/scripts/select_ros_version.sh ROS1 && \
	source devel/setup.bash && \
	source /home/kota/mid360_ws/devel/setup.bash
else
SETUP_CMD := \
	cd /home/kota/super_ws && \
	export DISABLE_ROS1_EOL_WARNINGS=1 && \
	source /opt/ros/noetic/setup.bash && \
	bash src/SUPER/scripts/select_ros_version.sh ROS1 && \
	catkin_make -DBUILD_TYPE=Release && \
	source devel/setup.bash && \
	source /home/kota/mid360_ws/devel/setup.bash
endif

build:
	docker build -t super .

build-no-cache:
	docker build --build-arg DUMMY=`date +%s` -t super .

run:
	docker run  -it \
			$(DOCKER_FLAGS) \
			-v /media/kkondo/kota_elements/super:/home/kota/data \
			--rm super

# Interactive shell inside the container with all mounts (for debugging)
# Usage: make shell
shell:
	@mkdir -p ../data
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data:/home/kota/data \
		-v /home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm super \
		/bin/bash

# Run the full benchmark pipeline: simulate + analyze + generate LaTeX table
# Usage: make run-benchmark [NUM_TRIALS=10]
# Data is saved to SUPER/data/<TIMESTAMP>/ and LaTeX table is updated at
# ~/paper_writing/DYNUS_v3/tables/static_benchmark.tex
run-benchmark:
	@mkdir -p ../data/$(TIMESTAMP)
	@echo "Running SUPER static benchmark ($(NUM_TRIALS) trials)..."
	@echo "  Data directory: SUPER/data/$(TIMESTAMP)/"
	@echo "  LaTeX output:   ~/paper_writing/DYNUS_v3/tables/static_benchmark.tex"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/$(TIMESTAMP):/home/kota/data \
		-v /home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			(roscore &) && sleep 5 && \
			python3 src/SUPER/docker/benchmarking.py $(NUM_TRIALS) 50.0 305.0,0.0,3.0 0.5 && \
			python3 src/SUPER/docker/analyze_static_benchmark.py \
				--data-dir /home/kota/data \
				--latex-path /home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex"
	@echo ""
	@echo "Benchmark complete. Data saved to: SUPER/data/$(TIMESTAMP)/"
	@echo "To re-analyze: make analyze-benchmark DATA_DIR=$(TIMESTAMP)"

# Run only the analysis on existing data (no simulation)
# Usage: make analyze-benchmark DATA_DIR=20260209_135000
analyze-benchmark:
	@if [ -z "$(DATA_DIR)" ]; then \
		echo "ERROR: DATA_DIR is required"; \
		echo "Usage: make analyze-benchmark DATA_DIR=YYYYMMDD_HHMMSS"; \
		echo ""; \
		echo "Available benchmark runs:"; \
		ls -d ../data/*/ 2>/dev/null | xargs -I{} basename {} || echo "  (none found)"; \
		exit 1; \
	fi
	@echo "Analyzing SUPER benchmark data: $(DATA_DIR)"
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/$(DATA_DIR):/home/kota/data \
		-v /home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			python3 src/SUPER/docker/analyze_static_benchmark.py \
				--data-dir /home/kota/data \
				--latex-path /home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex"

# ============================================================================
# Static forest benchmark: easy/medium/hard forest environments
# ============================================================================

# Host paths for dynus .world files and obstacle parameters
DYNUS_WORLDS := /home/kkondo/code/dynus_ws/src/dynus/worlds
DYNUS_OBSTACLES := /home/kkondo/code/dynus_ws/src/dynus/benchmark_data/static
STATIC_ENVS := easy_forest,medium_forest,hard_forest
PLANNERS := super_l2:static_forest_l2.yaml,super_linf:static_forest.yaml
PLANNER_NAMES := super_l2,super_linf

# Run the static forest benchmark: convert worlds to PCD, simulate, analyze
# Usage: make run-static-benchmark [NUM_TRIALS=10]
# Data is saved to SUPER/data/<TIMESTAMP>/{easy,medium,hard}_forest/
run-static-benchmark:
	@mkdir -p ../data/$(TIMESTAMP)
	@echo "=== SUPER Static Forest Benchmark ==="
	@echo "  Environments: $(STATIC_ENVS)"
	@echo "  Trials/env:   $(NUM_TRIALS)"
	@echo "  Data dir:     SUPER/data/$(TIMESTAMP)/"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/$(TIMESTAMP):/home/kota/data \
		-v $(DYNUS_WORLDS):/home/kota/dynus_worlds:ro \
		-v $(DYNUS_OBSTACLES):/home/kota/obstacle_data:ro \
		-v /home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			python3 src/SUPER/docker/world_to_pcd.py \
				/home/kota/dynus_worlds/easy_forest.world \
				src/SUPER/mars_uav_sim/perfect_drone_sim/pcd/easy_forest.pcd --ground && \
			python3 src/SUPER/docker/world_to_pcd.py \
				/home/kota/dynus_worlds/medium_forest.world \
				src/SUPER/mars_uav_sim/perfect_drone_sim/pcd/medium_forest.pcd --ground && \
			python3 src/SUPER/docker/world_to_pcd.py \
				/home/kota/dynus_worlds/hard_forest.world \
				src/SUPER/mars_uav_sim/perfect_drone_sim/pcd/hard_forest.pcd --ground && \
			(roscore &) && sleep 5 && \
			python3 src/SUPER/docker/benchmarking.py $(NUM_TRIALS) 50.0 105.0,0.0,2.0 0.5 \
				--envs $(STATIC_ENVS) \
				--launch_file static_forest_benchmark.launch \
				--planners $(PLANNERS) && \
			pip install 'numpy>=1.20' -q && \
			python3 src/SUPER/docker/analyze_static_benchmark.py \
				--data-dir /home/kota/data \
				--envs $(STATIC_ENVS) \
				--planners $(PLANNER_NAMES) \
				--obstacle-dir /home/kota/obstacle_data \
				--goal 105.0,0.0,2.0 \
				--v-max 5.0 --a-max 20.0 --j-max 100.0 \
				--latex-path /home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex"
	@echo ""
	@echo "Benchmark complete. Data saved to: SUPER/data/$(TIMESTAMP)/"
	@echo "To re-analyze: make analyze-static-benchmark DATA_DIR=$(TIMESTAMP)"

# Run only the analysis on existing static forest benchmark data
# Usage: make analyze-static-benchmark DATA_DIR=20260209_135000
analyze-static-benchmark:
	@if [ -z "$(DATA_DIR)" ]; then \
		echo "ERROR: DATA_DIR is required"; \
		echo "Usage: make analyze-static-benchmark DATA_DIR=YYYYMMDD_HHMMSS"; \
		echo ""; \
		echo "Available benchmark runs:"; \
		ls -d ../data/*/ 2>/dev/null | xargs -I{} basename {} || echo "  (none found)"; \
		exit 1; \
	fi
	@echo "Analyzing static forest benchmark data: $(DATA_DIR)"
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/$(DATA_DIR):/home/kota/data \
		-v $(DYNUS_OBSTACLES):/home/kota/obstacle_data:ro \
		-v /home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			pip install 'numpy>=1.20' -q && \
			python3 src/SUPER/docker/analyze_static_benchmark.py \
				--data-dir /home/kota/data \
				--envs $(STATIC_ENVS) \
				--planners $(PLANNER_NAMES) \
				--obstacle-dir /home/kota/obstacle_data \
				--goal 105.0,0.0,2.0 \
				--v-max 5.0 --a-max 20.0 --j-max 100.0 \
				--latex-path /home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex"

# ============================================================================
# Standardized benchmark: run SUPER optimizer on pregenerated dynus corridors
# ============================================================================

# Host paths for dynus data
DYNUS_ROOT := /home/kkondo/code/dynus_ws/src/dynus
SFC_DIR := $(DYNUS_ROOT)/data
PAPER_DIR := /home/kkondo/paper_writing

# Output directory for standardized benchmark CSVs (inside SUPER repo, gitignored)
STD_BENCH_DIR := $(SUPER_ROOT)/data/standardized_benchmark

# Run SUPER trajectory optimizer on all .mysco2 corridor files
# Builds inside Docker, runs the benchmark with both L2 and L-inf norms
# Usage: make run-standardized-benchmark
run-standardized-benchmark:
	@mkdir -p $(STD_BENCH_DIR)
	@echo "=== SUPER Standardized Benchmark (L2 + L-inf) ==="
	@echo "  SFC data:   $(SFC_DIR)"
	@echo "  Output CSVs: $(STD_BENCH_DIR)/super_l2_benchmark.csv"
	@echo "               $(STD_BENCH_DIR)/super_linf_benchmark.csv"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SFC_DIR):/home/kota/dynus_data:ro \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			echo '--- Running L2 norm benchmark ---' && \
			devel/lib/super_planner/local_traj_benchmark \
				--sfc_dir /home/kota/dynus_data \
				--config src/SUPER/super_planner/config/benchmark_l2.yaml \
				--output_csv src/SUPER/data/standardized_benchmark/super_l2_benchmark.csv \
				--dump_dir src/SUPER/data/standardized_benchmark/traj_l2 && \
			echo '--- Running L-inf norm benchmark ---' && \
			devel/lib/super_planner/local_traj_benchmark \
				--sfc_dir /home/kota/dynus_data \
				--config src/SUPER/super_planner/config/benchmark.yaml \
				--output_csv src/SUPER/data/standardized_benchmark/super_linf_benchmark.csv \
				--dump_dir src/SUPER/data/standardized_benchmark/traj_linf"
	@echo ""
	@echo "Benchmark complete. CSVs written to:"
	@echo "  $(STD_BENCH_DIR)/super_l2_benchmark.csv"
	@echo "  $(STD_BENCH_DIR)/super_linf_benchmark.csv"
	@echo ""
	@echo "Next: make analyze-standardized-benchmark"

# Analyze benchmark CSVs and update the SUPER rows in the LaTeX table
# Runs on the host (no Docker needed, no extra dependencies)
# Usage: make analyze-standardized-benchmark
analyze-standardized-benchmark:
	@echo "=== Analyzing Standardized Benchmark ==="
	@if [ ! -f "$(STD_BENCH_DIR)/super_l2_benchmark.csv" ]; then \
		echo "ERROR: super_l2_benchmark.csv not found."; \
		echo "Run 'make run-standardized-benchmark' first."; \
		exit 1; \
	fi
	@if [ ! -f "$(STD_BENCH_DIR)/super_linf_benchmark.csv" ]; then \
		echo "ERROR: super_linf_benchmark.csv not found."; \
		echo "Run 'make run-standardized-benchmark' first."; \
		exit 1; \
	fi
	python3 $(SUPER_ROOT)/docker/update_standardized_table.py \
		--csv-l2 $(STD_BENCH_DIR)/super_l2_benchmark.csv \
		--csv-linf $(STD_BENCH_DIR)/super_linf_benchmark.csv \
		--latex-path $(PAPER_DIR)/DYNUS_v3/tables/standardized_benchmark.tex
	@echo ""
	@echo "LaTeX table updated at:"
	@echo "  $(PAPER_DIR)/DYNUS_v3/tables/standardized_benchmark.tex"

# ============================================================================
# Video recording: run SUPER in hard_forest and capture rviz as MP4
# ============================================================================

# Duration in seconds (default 60)
VIDEO_DURATION ?= 60

# Record video with GUI (requires X11 display)
# Usage: make record-video [VIDEO_DURATION=60]
# Output: SUPER/data/video/<TIMESTAMP>/super_hard_forest_*.{bag,mp4}
record-video:
	@mkdir -p ../data/video/$(TIMESTAMP)
	@echo "=== SUPER Video Recording (hard_forest, GUI mode) ==="
	@echo "  Duration: $(VIDEO_DURATION)s"
	@echo "  Output:   SUPER/data/video/$(TIMESTAMP)/"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/video/$(TIMESTAMP):/home/kota/data \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			bash src/SUPER/docker/record_video.sh \
				--duration $(VIDEO_DURATION) \
				--output-dir /home/kota/data"
	@echo ""
	@echo "Recording complete. Files saved to:"
	@echo "  SUPER/data/video/$(TIMESTAMP)/"
	@ls -lh ../data/video/$(TIMESTAMP)/ 2>/dev/null || true

# Record video in headless mode (no display needed, uses Xvfb)
# Usage: make record-video-headless [VIDEO_DURATION=60]
record-video-headless:
	@mkdir -p ../data/video/$(TIMESTAMP)
	@echo "=== SUPER Video Recording (hard_forest, headless) ==="
	@echo "  Duration: $(VIDEO_DURATION)s"
	@echo "  Output:   SUPER/data/video/$(TIMESTAMP)/"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SUPER_ROOT)/data/video/$(TIMESTAMP):/home/kota/data \
		--rm super \
		/bin/bash -c "\
			apt-get update -qq && apt-get install -y -qq xvfb ffmpeg > /dev/null 2>&1 && \
			$(SETUP_CMD) && \
			bash src/SUPER/docker/record_video.sh \
				--headless \
				--duration $(VIDEO_DURATION) \
				--output-dir /home/kota/data"
	@echo ""
	@echo "Recording complete. Files saved to:"
	@echo "  SUPER/data/video/$(TIMESTAMP)/"
	@ls -lh ../data/video/$(TIMESTAMP)/ 2>/dev/null || true

# Replay an existing rosbag with rviz (for re-recording video or visualization)
# Usage: make replay-bag BAG_FILE=data/video/20260316_120000/super_hard_forest_20260316_120000.bag
replay-bag:
	@if [ -z "$(BAG_FILE)" ]; then \
		echo "ERROR: BAG_FILE is required"; \
		echo "Usage: make replay-bag BAG_FILE=path/to/file.bag"; \
		echo ""; \
		echo "Available bags:"; \
		find ../data -name "*.bag" 2>/dev/null | head -20 || echo "  (none found)"; \
		exit 1; \
	fi
	@echo "=== Replaying rosbag with rviz ==="
	docker run -it \
		$(DOCKER_FLAGS) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(realpath $(BAG_FILE)):/home/kota/replay.bag:ro \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			(roscore &) && sleep 3 && \
			rviz -d src/SUPER/mars_uav_sim/perfect_drone_sim/rviz/benchmark.rviz & \
			sleep 3 && \
			rosbag play /home/kota/replay.bag --clock && \
			echo 'Playback complete. Press Ctrl+C to exit.' && \
			wait"
