farol_clean() {
    if [[ -z "$COLCON_ROOT" ]]; then
        echo "Error: COLCON_ROOT is not set."
        return 1
    fi

    if [[ ! -d "$COLCON_ROOT" ]]; then
        echo "Error: COLCON_ROOT does not exist: $COLCON_ROOT"
        return 1
    fi

    # Safety checks so you don't nuke something stupid.
    case "$COLCON_ROOT" in
        "/"|"$HOME"|"/home"|"/usr"|"/opt"|"/tmp")
            echo "Error: refusing to clean dangerous COLCON_ROOT: $COLCON_ROOT"
            return 1
            ;;
    esac

    echo "Searching for build/install/log folders inside:"
    echo "  $COLCON_ROOT"
    echo

    local dirs
    dirs="$(
        find "$COLCON_ROOT" \
            -type d \( -name build -o -name install -o -name log \) \
            -prune \
            2>/dev/null
    )"

    if [[ -z "$dirs" ]]; then
        echo "No build/install/log folders found."
        return 0
    fi

    echo "The following folders will be removed:"
    echo "$dirs"
    echo

    read -r -p "Are you sure? [y/N] " answer

    case "$answer" in
        y|Y|yes|YES)
            echo "$dirs" | while IFS= read -r dir; do
                echo "Removing: $dir"
                rm -rf -- "$dir"
            done

            # reset ROS env in this terminal
            unset AMENT_PREFIX_PATH
            unset CMAKE_PREFIX_PATH
            unset COLCON_PREFIX_PATH

            source /opt/ros/jazzy/setup.bash

            echo "Done."
            ;;
        *)
            echo "Aborted."
            return 1
            ;;
    esac
}