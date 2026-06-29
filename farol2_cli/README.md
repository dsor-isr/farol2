# FAROL CLI

`farol` is a fast, pure Bash command dispatcher for FAROL development helpers.
It does not need `colcon build`, ROS package installation, Python imports, or
workspace sourcing for top-level dispatch or completion.

## Install

```bash
./install.sh
```

This symlinks:

```text
~/.local/bin/farol
~/.local/share/bash-completion/completions/farol
~/.local/share/farol/farol.bash
```

By default, `install.sh` also adds these idempotent lines to `~/.bashrc`:

```text
export PATH="$HOME/.local/bin:$PATH"
source "$HOME/.local/share/farol/farol.bash"
```

That makes `farol` and `farol cd ...` work in future Bash terminals. To install
only the symlinks and skip `.bashrc` edits:

```bash
./install.sh --no-modify-shell-rc
```

For a one-off shell without installing:

```bash
export PATH="$PWD/bin:$PATH"
source shell/farol.bash
```

`shell/farol.bash` loads Bash completion automatically.

`shell/farol.bash` also sources lightweight snippets from `profile.d/*.bash`.
Use that folder for aliases, prompt tweaks, environment variables, and small
shell functions that should always be available. Set
`FAROL_DISABLE_PROFILE=1` before sourcing `shell/farol.bash` to skip them.

## Uninstall

```bash
./uninstall.sh
```

## Commands

```bash
farol help
farol build
farol clean
farol source
farol ws status
farol ws root
farol pkg src farol2_nav
farol pkg share farol2_nav
farol cd planning
farol bridge serial /dev/ttyACM0
farol bag info my_bag
farol bag topics my_bag
farol bag play my_bag --rate 2
farol bag crop my_bag 10 20 -a
farol bag crop my_bag 10 20 '/magicelectric0/measurement/*'
farol kill ros
```

`farol cd ...` changes directory when the optional shell integration is sourced.
Without that integration, the executable prints a path and can still be used as:

```bash
cd "$(farol cd path_following)"
```

## Command Map

| User command | Backend script |
| --- | --- |
| `farol build` / `farol ws build` | `libexec/farol-build` |
| `farol clean` / `farol ws clean` | `libexec/farol-clean` |
| `farol source` | `libexec/farol-source` |
| `farol cd` | `libexec/farol-cd` |
| `farol pkg src` | `libexec/farol-pkg-src` |
| `farol pkg share` | `libexec/farol-pkg-share` |
| `farol pkg cd-src` | `libexec/farol-pkg-cd-src` |
| `farol bag crop` | `libexec/farol-bag-crop` |
| `farol bag info` | `libexec/farol-bag-info` |
| `farol bag play` | `libexec/farol-bag-play` |
| `farol bag topics` | `libexec/farol-bag-topics` |
| `farol bridge serial` / `farol serial bridge` | `libexec/farol-bridge-serial` |
| `farol kill ros` | `libexec/farol-kill-ros` |
| `farol ws status` | `libexec/farol-ws-status` |
| `farol ws root` | `libexec/farol-ws-root` |

## Completion

Bash completion does not call Python, ROS, colcon, or `farol`. It is loaded by
`shell/farol.bash` and also installed as a standard bash-completion file. Most
command completion is static. `farol cd` completion additionally scans
`$COLCON_ROOT/src/**/package.xml` with Bash globbing and suggests package
directory names with a leading `farol2_` stripped, so `farol2_planning`
completes as `planning`.
If your system does not load `~/.local/share/bash-completion/completions`
automatically, source it manually:

```bash
source /path/to/farol2_cli/completion/farol.bash
```

Zsh users can source:

```zsh
source /path/to/farol2_cli/completion/farol.zsh
```

## Migration Notes

The previous `farol2_cli` was an `ament_python` package with an argparse entry
point. That has been removed. The CLI now runs directly from `bin/farol` and
dispatches to `libexec/` scripts. Existing legacy functions in
`farol2_scripts/` remain available.

Legacy mapping:

| Old helper | New command |
| --- | --- |
| `farol_build` | `farol build` |
| `farol_clean` | `farol clean` |
| `crop_bag` | `farol bag crop` |
| `serial_bridge` | `farol bridge serial` |
| `nuke_ros2` | `farol kill ros` |

## Adding A New Command

To add:

```bash
farol foo bar
```

create:

```text
libexec/farol-foo-bar
```

Then add `foo` and `bar` to:

```text
bin/farol
completion/farol.bash
completion/farol.zsh
README.md
```

Keep dispatch and completion static. Put expensive work in the backend script.
Put always-sourced shell customizations in `profile.d/*.bash`, not in
`bin/`, `libexec/`, or `completion/`.

## Speed Design

`bin/farol` only parses one or two command words and `exec`s a backend. Static
completion avoids all dynamic discovery. Commands that need ROS or colcon call
those tools only after the concrete subcommand is selected.

## Manual Test Checklist

```bash
./bin/farol help
./bin/farol --help
./bin/farol invalid_command
./bin/farol bag help
./bin/farol kill help
grep -R "python" completion/
time ./bin/farol help
```
