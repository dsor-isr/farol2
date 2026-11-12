#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

rm -f -- "$HOME/.local/bin/farol"
rm -f -- "$HOME/.local/share/bash-completion/completions/farol"
rm -f -- "$HOME/.local/share/farol/farol.bash"
echo "Removed FAROL CLI symlinks from ~/.local."
