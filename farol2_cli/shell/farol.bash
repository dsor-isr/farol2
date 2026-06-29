# Optional shell integration for commands that need to affect this shell.
_farol_shell_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_farol_root_dir="$(cd -- "$_farol_shell_dir/.." && pwd)"

if [[ -f "$_farol_root_dir/completion/farol.bash" ]]; then
    # shellcheck source=/dev/null
    source "$_farol_root_dir/completion/farol.bash"
fi

if [[ "${FAROL_DISABLE_PROFILE:-0}" != "1" ]]; then
    for _farol_profile in "$_farol_root_dir"/profile.d/*.bash; do
        [[ -f "$_farol_profile" ]] || continue
        # shellcheck source=/dev/null
        source "$_farol_profile"
    done
    unset _farol_profile
fi

farol() {
    if [[ -n "${_ARGCOMPLETE-}" ]]; then
        command farol "$@"
        return
    fi

    case "${1-}" in
        cd)
            shift
            local destination
            destination="$(command farol cd "$@")" || return
            builtin cd -- "$destination"
            ;;
        source)
            shift
            local source_command
            source_command="$(command farol source "$@")" || return
            eval "$source_command"
            ;;
        *)
            command farol "$@"
        ;;
    esac
}

unset _farol_root_dir
unset _farol_shell_dir
