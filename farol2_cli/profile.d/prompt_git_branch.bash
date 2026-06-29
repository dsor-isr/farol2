# Display the current Git branch in the Bash prompt.
git_branch() {
    git branch --no-color 2>/dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1) /'
}

bash_color=32
case "$(hostname)" in
    mbot05h) bash_color=34 ;;
    mbot05n) bash_color=35 ;;
    harode-server) bash_color=36 ;;
esac

PS1='${debian_chroot:+($debian_chroot)}\[\033[01;${bash_color}m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\W\[\033[00;32m\] $(git_branch)\[\033[00m\]\$ '
