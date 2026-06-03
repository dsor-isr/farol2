nuke_ros2() {
    local patterns=(
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

    local self_pid="$$"
    local pids=""

    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -z "$pid" ]] && continue
            [[ "$pid" == "$self_pid" ]] && continue
            pids="$pids $pid"
        done < <(pgrep -f "$pattern" || true)
    done

    pids=$(echo "$pids" | tr ' ' '\n' | sort -u | tr '\n' ' ')

    if [[ -z "${pids// }" ]]; then
        echo "No ROS 2 processes found."
        return 0
    fi

    echo "Found ROS 2-related processes:"
    ps -fp $pids || true

    echo
    echo "Sending SIGINT..."
    kill -INT $pids 2>/dev/null || true
    sleep 2

    local remaining=""
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            remaining="$remaining $pid"
        fi
    done

    if [[ -n "${remaining// }" ]]; then
        echo "Sending SIGTERM..."
        kill -TERM $remaining 2>/dev/null || true
        sleep 2
    fi

    local remaining2=""
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            remaining2="$remaining2 $pid"
        fi
    done

    if [[ -n "${remaining2// }" ]]; then
        echo "Sending SIGKILL..."
        kill -KILL $remaining2 2>/dev/null || true
    fi

    echo "Stopping ROS 2 daemon..."
    ros2 daemon stop 2>/dev/null || true

    echo "Done."
}