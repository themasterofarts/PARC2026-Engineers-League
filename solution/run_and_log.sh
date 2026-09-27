#!/bin/bash
# Runs one full navigation-task benchmark — sim + task_solution.py — with a
# timestamped log, a rosbag recording of the key topics, and a one-line
# summary appended to run_history.md, for tracking tuning iterations over
# time. Run from inside the ROS 2 workspace container.
#
# Usage:
#   ./run_and_log.sh            # headless sim (fast, no GUI) — default
#   ./run_and_log.sh --gui      # official task.launch.py, with Gazebo/RViz GUI
# Note: no `set -u` — ROS 2's own setup.bash references unset variables
# internally (e.g. AMENT_TRACE_SETUP_FILES) and fails immediately under it.
set -o pipefail

cd "$(dirname "$0")"
SOLUTION_DIR="$(pwd)"
LOG_DIR="$SOLUTION_DIR/logs"
BAG_DIR="$SOLUTION_DIR/bags"
HISTORY_FILE="$SOLUTION_DIR/run_history.md"

mkdir -p "$LOG_DIR" "$BAG_DIR"

GUI=0
[ "${1:-}" = "--gui" ] && GUI=1

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/run_${TIMESTAMP}.log"
BAG_PATH="$BAG_DIR/run_${TIMESTAMP}"

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

LAUNCH_PKG="parc_nav_solution"
LAUNCH_FILE="task_headless.launch.py"
if [ "$GUI" -eq 1 ]; then
    LAUNCH_PKG="parc_robot_bringup"
    LAUNCH_FILE="task.launch.py"
fi

SIM_PID=""
BAG_PID=""

# `wait <pid>` has no timeout, and a `ros2 launch`/`ros2 bag record` process
# can fail to exit even after every one of its own child processes already
# has (observed in practice on both the nav2 launch and this sim launch) —
# an unbounded `wait` there hangs this script forever. Poll with `kill -0`
# instead so we always give up and move on.
wait_for_exit() {
    pid="$1"
    timeout_sec="$2"
    waited=0
    while kill -0 "$pid" 2>/dev/null; do
        if [ "$waited" -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 0
}

stop_process_group() {
    pid="$1"
    label="$2"
    kill -INT -"$pid" 2>/dev/null
    if wait_for_exit "$pid" 10; then
        return
    fi
    echo "cleanup: $label (pgid $pid) still alive after SIGINT, sending SIGKILL"
    kill -KILL -"$pid" 2>/dev/null
    if ! wait_for_exit "$pid" 10; then
        echo "cleanup: $label (pgid $pid) still alive after SIGKILL, giving up on it"
    fi
}

cleanup() {
    if [ -n "$BAG_PID" ]; then
        stop_process_group "$BAG_PID" "rosbag record"
    fi
    if [ -n "$SIM_PID" ]; then
        stop_process_group "$SIM_PID" "sim launch"
    fi
}
trap cleanup EXIT INT TERM

echo "Launching simulation ($LAUNCH_PKG $LAUNCH_FILE)..."
setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" > "$LOG_DIR/sim_${TIMESTAMP}.log" 2>&1 &
SIM_PID=$!

echo "Waiting for the robot to spawn..."
for i in $(seq 1 60); do
    ros2 topic list 2>/dev/null | grep -q "^/odom$" && break
    sleep 1
done
sleep 2  # let topics settle before recording starts

echo "Starting rosbag recording -> $BAG_PATH"
setsid ros2 bag record -o "$BAG_PATH" \
    /odom /scan /tf /tf_static /robot_base_controller/cmd_vel_unstamped \
    /base_collisions /top_chassis_collisions /left_wheel_collisions /right_wheel_collisions \
    > "$LOG_DIR/rosbag_${TIMESTAMP}.log" 2>&1 &
BAG_PID=$!

echo "Running task_solution.py -> $LOG_FILE"
START_TIME=$(date +%s)
ros2 run parc_nav_solution task_solution.py 2>&1 | tee "$LOG_FILE"
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

RESULT="UNKNOWN"
grep -q "Goal reached" "$LOG_FILE" && RESULT="SUCCEEDED"
grep -q "Navigation failed" "$LOG_FILE" && RESULT="FAILED"
grep -q "Navigation canceled" "$LOG_FILE" && RESULT="CANCELED"

cleanup
trap - EXIT INT TERM

MIN=$((ELAPSED / 60))
SEC=$((ELAPSED % 60))
echo "Result: $RESULT — elapsed ${MIN}m${SEC}s"

if [ ! -f "$HISTORY_FILE" ]; then
    {
        echo "# Run history"
        echo
        echo "| Timestamp | Result | Elapsed | Log | Bag |"
        echo "|---|---|---|---|---|"
    } > "$HISTORY_FILE"
fi
echo "| $TIMESTAMP | $RESULT | ${MIN}m ${SEC}s | logs/run_${TIMESTAMP}.log | bags/run_${TIMESTAMP} |" >> "$HISTORY_FILE"

echo "Logged to $HISTORY_FILE"
