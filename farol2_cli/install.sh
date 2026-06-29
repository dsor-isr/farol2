#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BIN_DIR="$HOME/.local/bin"
BASH_COMPLETION_DIR="$HOME/.local/share/bash-completion/completions"
BASH_INTEGRATION_DIR="$HOME/.local/share/farol"
MODIFY_RC=true

for arg in "$@"; do
    case "$arg" in
        --modify-shell-rc) MODIFY_RC=true ;;
        --no-modify-shell-rc) MODIFY_RC=false ;;
        -h|--help)
            echo "Usage: ./install.sh [--modify-shell-rc|--no-modify-shell-rc]"
            exit 0
            ;;
        *) echo "install.sh: unknown option: $arg" >&2; exit 1 ;;
    esac
done

mkdir -p "$BIN_DIR" "$BASH_COMPLETION_DIR" "$BASH_INTEGRATION_DIR"
ln -sfn "$SCRIPT_DIR/bin/farol" "$BIN_DIR/farol"
ln -sfn "$SCRIPT_DIR/completion/farol.bash" "$BASH_COMPLETION_DIR/farol"
ln -sfn "$SCRIPT_DIR/shell/farol.bash" "$BASH_INTEGRATION_DIR/farol.bash"

echo "Installed:"
echo "  $BIN_DIR/farol -> $SCRIPT_DIR/bin/farol"
echo "  $BASH_COMPLETION_DIR/farol -> $SCRIPT_DIR/completion/farol.bash"
echo "  $BASH_INTEGRATION_DIR/farol.bash -> $SCRIPT_DIR/shell/farol.bash"
echo

if [[ "$MODIFY_RC" == true ]]; then
    line='export PATH="$HOME/.local/bin:$PATH"'
    if ! grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null; then
        printf '\n%s\n' "$line" >> "$HOME/.bashrc"
        echo "Added ~/.local/bin to ~/.bashrc."
    fi
    source_line='source "$HOME/.local/share/farol/farol.bash"'
    if ! grep -Fqx "$source_line" "$HOME/.bashrc" 2>/dev/null; then
        printf '%s\n' "$source_line" >> "$HOME/.bashrc"
        echo "Added FAROL shell integration to ~/.bashrc."
    fi
else
    echo "Skipped ~/.bashrc changes."
    echo "Make sure ~/.local/bin is in PATH."
    echo "For 'farol cd ...' and 'farol source', add this to ~/.bashrc:"
    echo "  source $BASH_INTEGRATION_DIR/farol.bash"
fi
