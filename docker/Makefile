.PHONY: build build-no-cache run run-benchmark analyze-benchmark shell \
       run-standardized-benchmark analyze-standardized-benchmark \
       run-static-benchmark analyze-static-benchmark

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
BENCHMARK_DATA_DIR := $(DYNUS_ROOT)/benchmark_data
PAPER_DIR := /home/kkondo/paper_writing

# Run SUPER trajectory optimizer on all .mysco2 corridor files
# Builds inside Docker, runs the benchmark, writes CSV output
# Usage: make run-standardized-benchmark
run-standardized-benchmark:
	@echo "=== SUPER Standardized Benchmark ==="
	@echo "  SFC data:   $(SFC_DIR)"
	@echo "  Output CSV: $(BENCHMARK_DATA_DIR)/default/super_benchmark.csv"
	@echo ""
	docker run -it \
		$(DOCKER_FLAGS) \
		-v $(SUPER_ROOT):/home/kota/super_ws/src/SUPER:rw \
		-v $(SFC_DIR):/home/kota/dynus_data:ro \
		-v $(BENCHMARK_DATA_DIR):/home/kota/benchmark_output:rw \
		--rm super \
		/bin/bash -c "\
			$(SETUP_CMD) && \
			devel/lib/super_planner/local_traj_benchmark \
				--sfc_dir /home/kota/dynus_data \
				--config src/SUPER/super_planner/config/benchmark.yaml \
				--output_csv /home/kota/benchmark_output/default/super_benchmark.csv"
	@echo ""
	@echo "Benchmark complete. CSV written to:"
	@echo "  $(BENCHMARK_DATA_DIR)/default/super_benchmark.csv"
	@echo ""
	@echo "Next: make analyze-standardized-benchmark"

# Analyze benchmark CSV and update the SUPER row in the LaTeX table
# Runs on the host (no Docker needed, no extra dependencies)
# Usage: make analyze-standardized-benchmark
analyze-standardized-benchmark:
	@echo "=== Analyzing Standardized Benchmark ==="
	@if [ ! -f "$(BENCHMARK_DATA_DIR)/default/super_benchmark.csv" ]; then \
		echo "ERROR: super_benchmark.csv not found."; \
		echo "Run 'make run-standardized-benchmark' first."; \
		exit 1; \
	fi
	python3 $(SUPER_ROOT)/docker/update_standardized_table.py \
		--csv $(BENCHMARK_DATA_DIR)/default/super_benchmark.csv \
		--latex-path $(PAPER_DIR)/DYNUS_v3/tables/standardized_benchmark.tex
	@echo ""
	@echo "LaTeX table updated at:"
	@echo "  $(PAPER_DIR)/DYNUS_v3/tables/standardized_benchmark.tex"
