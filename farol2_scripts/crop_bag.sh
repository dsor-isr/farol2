crop_bag() {
    local all_topics=false
    local out_bag=""

    while getopts ":ao:" opt; do
        case "$opt" in
            a) all_topics=true ;;
            o) out_bag="$OPTARG" ;;
            *)
                echo "Usage:"
                echo "  crop_bag [-a] [-o output_bag] <input_bag> <t1_sec> <t2_sec> [topics_or_patterns...]"
                return 1
                ;;
        esac
    done
    shift $((OPTIND - 1))
    OPTIND=1

    if [ "$#" -lt 3 ]; then
        echo "Usage:"
        echo "  crop_bag [-a] [-o output_bag] <input_bag> <t1_sec> <t2_sec> [topics_or_patterns...]"
        echo
        echo "Examples:"
        echo "  crop_bag my_bag 10 20 /imu/data /gps/fix"
        echo "  crop_bag my_bag 10 20 '/magicelectric0/measurement/*'"
        echo "  crop_bag -a my_bag 10 20"
        echo "  crop_bag -o my_crop my_bag 10 20 /imu/data"
        return 1
    fi

    local in_bag="$1"
    local t1="$2"
    local t2="$3"
    shift 3

    local requested_topics=("$@")

    [ -z "$out_bag" ] && out_bag="${in_bag}_crop_${t1}_${t2}"

    if [ ! -e "$in_bag" ]; then
        echo "Input bag does not exist: $in_bag"
        return 1
    fi

    if (( $(echo "$t2 <= $t1" | bc -l) )); then
        echo "Error: t2 must be greater than t1"
        return 1
    fi

    if [ "$all_topics" = false ] && [ "${#requested_topics[@]}" -eq 0 ]; then
        echo "Error: specify topics/patterns or use -a for all topics."
        return 1
    fi

    local start_sec
    start_sec=$(ros2 bag info "$in_bag" | grep "Start:" | sed -E 's/.*\(([0-9]+(\.[0-9]+)?)\).*/\1/')

    if [ -z "$start_sec" ]; then
        echo "Could not extract bag start time from ros2 bag info."
        return 1
    fi

    local start_ns end_ns
    start_ns=$(python3 -c "print(int((${start_sec} + ${t1}) * 1e9))")
    end_ns=$(python3 -c "print(int((${start_sec} + ${t2}) * 1e9))")

    local topics=()

    if [ "$all_topics" = false ]; then
        local available_topics
        available_topics=$(ros2 bag info "$in_bag" | grep "Topic:" | sed -E 's/.*Topic: ([^ ]+).*/\1/')

        for req in "${requested_topics[@]}"; do
            if [[ "$req" == *"*"* ]]; then
                while read -r topic; do
                    [[ -z "$topic" ]] && continue
                    if [[ "$topic" == $req ]]; then
                        topics+=("$topic")
                    fi
                done <<< "$available_topics"
            else
                topics+=("$req")
            fi
        done

        mapfile -t topics < <(printf "%s\n" "${topics[@]}" | sort -u)

        if [ "${#topics[@]}" -eq 0 ]; then
            echo "Error: no topics matched."
            return 1
        fi
    fi

    local tmpdir
    tmpdir=$(mktemp -d)

    cat > "$tmpdir/output.yaml" <<EOF
output_bags:
- uri: $out_bag
  storage_id: mcap
  start_time_ns: $start_ns
  end_time_ns: $end_ns
EOF

    if [ "$all_topics" = true ]; then
        cat >> "$tmpdir/output.yaml" <<EOF
  all_topics: true
  all_services: true
  all_actions: true
EOF
    else
        echo "  topics:" >> "$tmpdir/output.yaml"
        for topic in "${topics[@]}"; do
            echo "  - $topic" >> "$tmpdir/output.yaml"
        done
    fi

    echo "Cropping bag:"
    echo "  input:   $in_bag"
    echo "  output:  $out_bag"
    echo "  storage: mcap"
    echo "  window:  ${t1}s -> ${t2}s relative to bag start"

    if [ "$all_topics" = true ]; then
        echo "  topics:  all"
    else
        echo "  topics:"
        for topic in "${topics[@]}"; do
            echo "    $topic"
        done
    fi

    echo

    ros2 bag convert \
        -i "$in_bag" mcap \
        -o "$tmpdir/output.yaml"

    local ret=$?
    rm -rf "$tmpdir"
    return $ret
}