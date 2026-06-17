#!/usr/bin/env bash

farol_build() {
    if [[ -z "$COLCON_ROOT" ]]; then
        echo "Error: COLCON_ROOT is not set."
        return 1
    fi

    if [[ ! -d "$COLCON_ROOT" ]]; then
        echo "Error: COLCON_ROOT does not exist: $COLCON_ROOT"
        return 1
    fi

    local nproc_total
    local nproc_build

    nproc_total="$(nproc)"

    # Leave 2 cores free if possible, otherwise leave 1.
    if (( nproc_total > 4 )); then
        nproc_build=$((nproc_total - 2))
    elif (( nproc_total > 1 )); then
        nproc_build=$((nproc_total - 1))
    else
        nproc_build=1
    fi

    cd "$COLCON_ROOT" || return 1

    echo "Building in: $PWD"
    echo "Using $nproc_build parallel workers out of $nproc_total cores..."

    colcon build \
        --symlink-install \
        --parallel-workers "$nproc_build" \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

    local build_status=$?

    cd - >/dev/null || return 1

    return "$build_status"
}