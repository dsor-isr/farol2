# FAROL Profile Snippets

Put lightweight Bash snippets here when they should be sourced with
`shell/farol.bash`.

Good fits:

- aliases
- prompt tweaks
- small shell functions
- environment variables

Avoid slow startup work here:

- Python
- ROS commands
- colcon commands
- expensive filesystem scans

Files must end in `.bash` to be sourced automatically.

Set this before sourcing `shell/farol.bash` to skip all snippets:

```bash
export FAROL_DISABLE_PROFILE=1
```
