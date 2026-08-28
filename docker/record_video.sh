#!/usr/bin/env bash
# record_video.sh — Run SUPER in hard_forest, record rosbag + screen video
#
# Usage (inside Docker):
#   bash record_video.sh [--headless] [--duration 60] [--output-dir /home/kota/data]
#
# The script:
#   1. Starts roscore
#   2. Launches the static_forest_benchmark with hard_forest + rviz
#   3. Records a rosbag of all topics
#   4. Publishes the goal after a delay
#   5. Captures the rviz window as an MP4 video via ffmpeg
#   6. Stops everything after goal is reached or timeout
set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
HEADLESS=0
DURATION=60
OUTPUT_DIR="/home/kota/data"
PLANNER_CONFIG="static_forest.yaml"
DRONE_CONFIG="static_forest_hard_forest.yaml"
GOAL="105.0,0.0,2.0"
SCREEN_SIZE="1920x1080"
FPS=30

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless)   HEADLESS=1; shift ;;
        --duration)   DURATION="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --planner-config) PLANNER_CONFIG="$2"; shift 2 ;;
        --drone-config) DRONE_CONFIG="$2"; shift 2 ;;
        --goal)       GOAL="$2"; shift 2 ;;
        --screen-size) SCREEN_SIZE="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

IFS=',' read -r GOAL_X GOAL_Y GOAL_Z <<< "$GOAL"
mkdir -p "$OUTPUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_FILE="$OUTPUT_DIR/super_hard_forest_${TIMESTAMP}.bag"
VIDEO_FILE="$OUTPUT_DIR/super_hard_forest_${TIMESTAMP}.mp4"

echo "============================================="
echo "  SUPER Video Recording"
echo "============================================="
echo "  Environment: hard_forest"
echo "  Planner:     $PLANNER_CONFIG"
echo "  Goal:        ($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "  Duration:    ${DURATION}s"
echo "  Headless:    $HEADLESS"
echo "  Bag file:    $BAG_FILE"
echo "  Video file:  $VIDEO_FILE"
echo "============================================="

# ── Cleanup trap ──────────────────────────────────────────────────────────────
PIDS=()
cleanup() {
    echo ""
    echo "Cleaning up..."
    # Stop ffmpeg gracefully (send 'q')
    if [[ -n "${FFMPEG_PID:-}" ]] && kill -0 "$FFMPEG_PID" 2>/dev/null; then
        kill -INT "$FFMPEG_PID" 2>/dev/null || true
        wait "$FFMPEG_PID" 2>/dev/null || true
    fi
    # Stop rosbag record gracefully
    if [[ -n "${ROSBAG_PID:-}" ]] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        sleep 2
        wait "$ROSBAG_PID" 2>/dev/null || true
    fi
    # Kill remaining processes
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Kill Xvfb if we started it
    if [[ -n "${XVFB_PID:-}" ]] && kill -0 "$XVFB_PID" 2>/dev/null; then
        kill "$XVFB_PID" 2>/dev/null || true
    fi
    sleep 1
    echo "Bag saved to:   $BAG_FILE"
    echo "Video saved to: $VIDEO_FILE"
    echo "Done."
}
trap cleanup EXIT

# ── Source ROS ────────────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1
source /home/kota/super_ws/devel/setup.bash
source /home/kota/mid360_ws/devel/setup.bash 2>/dev/null || true

# ── Virtual framebuffer for headless mode ─────────────────────────────────────
if [[ "$HEADLESS" -eq 1 ]]; then
    echo "Starting Xvfb (headless mode)..."
    Xvfb :99 -screen 0 "${SCREEN_SIZE}x24" &
    XVFB_PID=$!
    export DISPLAY=:99
    sleep 2
fi

# ── Start roscore ─────────────────────────────────────────────────────────────
echo "Starting roscore..."
roscore &
PIDS+=($!)
sleep 3

# ── Start rosbag recording ───────────────────────────────────────────────────
echo "Starting rosbag recording..."
rosbag record -a -O "$BAG_FILE" &
ROSBAG_PID=$!
PIDS+=($ROSBAG_PID)
sleep 1

# ── Launch simulation + rviz ──────────────────────────────────────────────────
echo "Launching simulation (hard_forest)..."
roslaunch --wait mission_planner static_forest_benchmark.launch \
    drone_config:="$DRONE_CONFIG" \
    planner_config:="$PLANNER_CONFIG" &
PIDS+=($!)
sleep 8

# ── Start video recording via ffmpeg ──────────────────────────────────────────
echo "Starting video recording..."
ffmpeg -y -video_size "$SCREEN_SIZE" -framerate "$FPS" \
    -f x11grab -i "$DISPLAY" \
    -c:v libx264 -preset fast -crf 23 -pix_fmt yuv420p \
    "$VIDEO_FILE" &
FFMPEG_PID=$!
sleep 2

# ── Publish goal ──────────────────────────────────────────────────────────────
echo "Publishing goal: ($GOAL_X, $GOAL_Y, $GOAL_Z)..."
rostopic pub /goal geometry_msgs/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1 &
PIDS+=($!)

# ── Wait for completion ───────────────────────────────────────────────────────
echo "Monitoring for ${DURATION}s (or until goal reached)..."
START_TIME=$(date +%s)
while true; do
    ELAPSED=$(( $(date +%s) - START_TIME ))
    if [[ "$ELAPSED" -ge "$DURATION" ]]; then
        echo "Timeout reached (${DURATION}s)."
        break
    fi

    # Check if drone reached the goal via rostopic echo
    POS=$(rostopic echo -n 1 /planning/pos_cmd/position 2>/dev/null || true)
    if [[ -n "$POS" ]]; then
        PX=$(echo "$POS" | grep "^x:" | awk '{print $2}')
        PY=$(echo "$POS" | grep "^y:" | awk '{print $2}')
        PZ=$(echo "$POS" | grep "^z:" | awk '{print $2}')
        if [[ -n "$PX" && -n "$PY" && -n "$PZ" ]]; then
            DIST=$(python3 -c "import math; print(f'{math.sqrt(($PX-$GOAL_X)**2+($PY-$GOAL_Y)**2+($PZ-$GOAL_Z)**2):.2f}')" 2>/dev/null || echo "999")
            echo "  [${ELAPSED}s] Distance to goal: ${DIST}m"
            if python3 -c "exit(0 if float('$DIST') < 0.5 else 1)" 2>/dev/null; then
                echo "Goal reached!"
                sleep 3  # Record a few more seconds
                break
            fi
        fi
    fi
    sleep 2
done

echo "Stopping recording..."
# cleanup is handled by the trap
