# FAROL2 CLI

`farol2_cli` exposes the existing FAROL2 shell helpers through a ROS 2-style
`farol2` command. The original scripts in `farol2_scripts` remain available
during migration.

## Build

From the colcon workspace root:

```bash
colcon build --symlink-install --packages-select farol2_cli
source install/setup.bash
```

## Bash integration and completion

Sourcing the workspace automatically loads the Bash wrapper so `farol2 cd`
can change the current directory. The wrapper also enables `argcomplete` when
it is installed:

```bash
sudo apt install python3-argcomplete
source install/setup.bash
```

The existing `farol2_scripts/source_all.sh` loader also loads the wrapper.
Without either setup script, `farol2 cd ...` only prints the resolved
destination because an executable cannot change its parent shell's directory.

Completion for `farol2 cd` includes workspace and installed ROS packages whose
names start with `farol2_`, both in full and shortened form. For example,
`farol2_path_following` also completes as `path_following`.

## Examples

```bash
farol2 --help
farol2 ws build
farol2 ws build -p farol2_nav farol2_inner_loop --release
farol2 ws clean
farol2 ws root
farol2 cd
farol2 cd root
farol2 cd drivers
farol2 cd farol2_nav
farol2 pkg src farol2_nav
cd $(farol2 pkg src farol2_nav)
farol2 pkg cd-src farol2_nav
farol2 serial bridge /dev/ttyACM0
farol2 bag info my_bag
farol2 bag play my_bag -- --rate 2
farol2 bag crop my_bag 10 20 -a
farol2 bag crop my_bag 10 20 '/magicelectric0/measurement/*'
farol2 kill ros
```

Arguments following `--` for `farol2 ws build` and `farol2 bag play` are passed
to the underlying command.

`farol2 ws clean` preserves `build/farol2_cli`, `install/farol2_cli`, and the
top-level install setup scripts. Preserving the build artifact is necessary
because a symlink-installed Python package refers back to it. Cleanup refuses
to modify an install space where `farol2_cli` cannot be preserved safely.

## Migration

| Existing helper | New command |
| --- | --- |
| `farol_build` | `farol2 ws build` |
| `farol_clean` | `farol2 ws clean` |
| `crop_bag` | `farol2 bag crop` |
| `serial_bridge` | `farol2 serial bridge` |
| `nuke_ros2` | `farol2 kill ros` |

`source_all.sh` and `display_git_branch_in_prompt.sh` remain shell-only helpers.
There were no existing Docker helpers, so `farol2 docker` currently documents
that no commands have been migrated rather than inventing new behavior.
