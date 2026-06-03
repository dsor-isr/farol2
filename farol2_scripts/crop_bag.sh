crop_bag_rel() {
    if [ "$#" -lt 3 ]; then
        echo "Usage:"
        echo "  crop_bag_rel <input_bag> <t1_sec> <t2_sec> [output_bag] [storage_id]"
        return 1
    fi

    local in_bag="$1"
    local t1="$2"
    local t2="$3"
    local out_bag="${4:-${in_bag}_crop_${t1}_${t2}}"
    local storage_id="${5:-mcap}"

    if [ ! -e "$in_bag" ]; then
        echo "Input bag does not exist: $in_bag"
        return 1
    fi

    if (( $(echo "$t2 <= $t1" | bc -l) )); then
        echo "Error: t2 must be greater than t1"
        return 1
    fi

    local start_sec
    start_sec=$(ros2 bag info "$in_bag" | grep "Start:" | sed -E 's/.*\(([0-9]+(\.[0-9]+)?)\).*/\1/')

    if [ -z "$start_sec" ]; then
        echo "Could not extract bag start time from ros2 bag info."
        return 1
    fi

    local start_ns end_ns
    start_ns=$(python3 - <<EOF
print(int((${start_sec} + ${t1}) * 1e9))
EOF
)

    end_ns=$(python3 - <<EOF
print(int((${start_sec} + ${t2}) * 1e9))
EOF
)

    local tmpdir
    tmpdir=$(mktemp -d)

    cat > "$tmpdir/output.yaml" <<EOF
output_bags:
- uri: $out_bag
  storage_id: $storage_id
  all_topics: true
  all_services: true
  all_actions: true
  start_time_ns: $start_ns
  end_time_ns: $end_ns
EOF

    echo "Cropping bag:"
    echo "  input:  $in_bag"
    echo "  output: $out_bag"
    echo "  window: ${t1}s -> ${t2}s relative to bag start"
    echo

    ros2 bag convert \
        -i "$in_bag" "$storage_id" \
        -o "$tmpdir/output.yaml"

    rm -rf "$tmpdir"
}

