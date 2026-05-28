#!/usr/bin/env bash
set -euo pipefail

# Kills ROS 2-related processes:
# 1. SIGINT  = like Ctrl-C
# 2. SIGTERM = polite terminate
# 3. SIGKILL = forced kill

PATTERNS=(
  "ros2"
  "ros2 launch"
  "launch_ros"
  "rviz2"
  "rqt"
  "component_container"
  "robot_state_publisher"
  "joint_state_publisher"
  "/opt/ros/.*/lib/"
  "/install/.*/lib/"
)

SELF_PID=$$
PIDS=""

for pattern in "${PATTERNS[@]}"; do
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ "$pid" == "$SELF_PID" ]] && continue
    PIDS="$PIDS $pid"
  done < <(pgrep -f "$pattern" || true)
done

# Remove duplicates
PIDS=$(echo "$PIDS" | tr ' ' '\n' | sort -u | tr '\n' ' ')

if [[ -z "${PIDS// }" ]]; then
  echo "No ROS 2 processes found."
  exit 0
fi

echo "Found ROS 2-related processes:"
ps -fp $PIDS || true

echo
echo "Sending SIGINT..."
kill -INT $PIDS 2>/dev/null || true
sleep 2

REMAINING=""
for pid in $PIDS; do
  if kill -0 "$pid" 2>/dev/null; then
    REMAINING="$REMAINING $pid"
  fi
done

if [[ -n "${REMAINING// }" ]]; then
  echo "Sending SIGTERM..."
  kill -TERM $REMAINING 2>/dev/null || true
  sleep 2
fi

REMAINING2=""
for pid in $PIDS; do
  if kill -0 "$pid" 2>/dev/null; then
    REMAINING2="$REMAINING2 $pid"
  fi
done

if [[ -n "${REMAINING2// }" ]]; then
  echo "Sending SIGKILL..."
  kill -KILL $REMAINING2 2>/dev/null || true
fi

echo "Stopping ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo "Done."
