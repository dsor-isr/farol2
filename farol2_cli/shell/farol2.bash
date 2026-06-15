# Shell integration for commands that must affect the calling Bash process.
farol2() {
    if [[ -n "${_ARGCOMPLETE-}" ]]; then
        command farol2 "$@"
        return
    fi

    if [[ "${1-}" == "cd" ]]; then
        local destination
        destination="$(command farol2 cd "${@:2}")" || return
        builtin cd -- "$destination"
        return
    fi

    command farol2 "$@"
}

if command -v register-python-argcomplete >/dev/null 2>&1; then
    eval "$(register-python-argcomplete farol2)"
fi
