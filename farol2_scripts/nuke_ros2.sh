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

    # Anything matching these will be protected.
    local exclude_patterns=(
        "plotjuggler"
        "PlotJuggler"
        "rqt"
        "foxglove_bridge"
        "foxglove[-_ ]bridge"
        "bag_record"
    )

    local self_pid="$$"
    local pids=""

    is_excluded_pid() {
        local pid="$1"
        local cmdline

        cmdline="$(ps -p "$pid" -o args= 2>/dev/null || true)"

        for exclude in "${exclude_patterns[@]}"; do
            if [[ "$cmdline" =~ $exclude ]]; then
                return 0
            fi
        done

        return 1
    }

    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -z "$pid" ]] && continue
            [[ "$pid" == "$self_pid" ]] && continue

            if is_excluded_pid "$pid"; then
                continue
            fi

            pids="$pids $pid"
        done < <(pgrep -f "$pattern" || true)
    done

    pids=$(echo "$pids" | tr ' ' '\n' | sort -u | tr '\n' ' ')

    if [[ -z "${pids// }" ]]; then
        echo "No ROS 2 processes found."
        return 0
    fi

    echo "Found ROS 2-related processes to kill:"
    ps -fp $pids || true

    echo
    echo "Protected processes matching:"
    printf '  - %s\n' "${exclude_patterns[@]}"

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