_farol_prompt_profile="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../farol2_cli/profile.d/prompt_git_branch.bash"
if [[ -f "$_farol_prompt_profile" ]]; then
    # shellcheck source=/dev/null
    source "$_farol_prompt_profile"
fi
unset _farol_prompt_profile
