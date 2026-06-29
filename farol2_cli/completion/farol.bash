_farol_completion_is_workspace_root() {
    local root="$1"
    [[ -d "$root/src" ]] || return 1
    compgen -G "$root/src/*/package.xml" >/dev/null && return 0
    compgen -G "$root/src/*/*/package.xml" >/dev/null && return 0
    return 1
}

_farol_completion_workspace_root() {
    if [[ -n "${COLCON_ROOT:-}" ]] && _farol_completion_is_workspace_root "$COLCON_ROOT"; then
        printf '%s\n' "$COLCON_ROOT"
        return 0
    fi

    local current
    current="$PWD"
    while [[ "$current" != "/" ]]; do
        if _farol_completion_is_workspace_root "$current"; then
            printf '%s\n' "$current"
            return 0
        fi
        if [[ "${current##*/}" == "src" ]] && _farol_completion_is_workspace_root "${current%/*}"; then
            printf '%s\n' "${current%/*}"
            return 0
        fi
        current="${current%/*}"
        [[ -n "$current" ]] || current="/"
    done
    return 1
}

_farol_completion_path_is_ignored() {
    local path="$1"
    local stop="$2"
    local current

    current="$(cd -- "$path" 2>/dev/null && pwd)" || return 1
    stop="$(cd -- "$stop" 2>/dev/null && pwd)" || return 1

    while [[ "$current" != "/" ]]; do
        [[ -e "$current/COLCON_IGNORE" ]] && return 0
        [[ "$current" == "$stop" ]] && break
        current="${current%/*}"
        [[ -n "$current" ]] || current="/"
    done

    return 1
}

_farol_complete_cd_targets() {
    local root package_xml package_dir package_name words
    root="$(_farol_completion_workspace_root)" || return 0

    words="root"
    local old_nullglob old_globstar
    old_nullglob="$(shopt -p nullglob)"
    old_globstar="$(shopt -p globstar)"
    shopt -s nullglob globstar
    for package_xml in "$root"/src/**/package.xml; do
        package_dir="${package_xml%/package.xml}"
        _farol_completion_path_is_ignored "$package_dir" "$root/src" && continue
        package_name="${package_dir##*/}"
        package_name="${package_name#farol2_}"
        words+=" $package_name"
    done
    eval "$old_nullglob"
    eval "$old_globstar"

    COMPREPLY=( $(compgen -W "$words" -- "$cur") )
}

_farol_complete() {
    local cur prev words cword
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    if (( COMP_CWORD == 1 )); then
        COMPREPLY=( $(compgen -W "build clean source cd pkg bag bridge serial drivers kill ws help" -- "$cur") )
        return 0
    fi

    case "${COMP_WORDS[1]}" in
        bag)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "crop info play topics help" -- "$cur") )
            fi
            ;;
        bridge)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "serial help" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "serial" ]]; then
                case "$prev" in
                    --link|--baud) COMPREPLY=() ;;
                    *) COMPREPLY=( $(compgen -W "--link --baud --help" -- "$cur") ) ;;
                esac
            fi
            ;;
        drivers)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "disable enable set status profiles help" -- "$cur") )
            elif [[ "${COMP_WORDS[2]}" == "set" && $COMP_CWORD -eq 3 ]]; then
                COMPREPLY=( $(compgen -W "all magicelectric" -- "$cur") )
            fi
            ;;
        serial)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "bridge help" -- "$cur") )
            fi
            ;;
        kill)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "ros help" -- "$cur") )
            fi
            ;;
        cd)
            if (( COMP_CWORD == 2 )); then
                _farol_complete_cd_targets
            fi
            ;;
        pkg)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "src share cd-src help" -- "$cur") )
            fi
            ;;
        ws)
            if (( COMP_CWORD == 2 )); then
                COMPREPLY=( $(compgen -W "status root build clean help" -- "$cur") )
            fi
            ;;
        build)
            case "$prev" in
                -p|--packages|-j|--cores) COMPREPLY=() ;;
                *) COMPREPLY=( $(compgen -W "-p --packages -j --cores --release --help" -- "$cur") ) ;;
            esac
            ;;
        clean)
            COMPREPLY=( $(compgen -W "-y --yes --help" -- "$cur") )
            ;;
    esac
}

complete -F _farol_complete farol
