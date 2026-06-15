#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "This script must be sourced:"
    echo "  source ${BASH_SOURCE[0]}"
    exit 1
fi

_farol2_scripts_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_farol2_scripts_loader="$_farol2_scripts_dir/$(basename "${BASH_SOURCE[0]}")"

for _farol2_script in "$_farol2_scripts_dir"/*.sh; do
    [[ -f "$_farol2_script" ]] || continue
    [[ "$_farol2_script" == "$_farol2_scripts_loader" ]] && continue

    # shellcheck source=/dev/null
    source "$_farol2_script"
done

_farol2_cli_shell="$_farol2_scripts_dir/../farol2_cli/shell/farol2.bash"
if [[ -f "$_farol2_cli_shell" ]]; then
    # shellcheck source=/dev/null
    source "$_farol2_cli_shell"
fi

unset _farol2_cli_shell
unset _farol2_script
unset _farol2_scripts_dir
unset _farol2_scripts_loader
