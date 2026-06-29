#compdef farol

_farol() {
  local -a commands bag_sub bridge_sub kill_sub pkg_sub ws_sub
  commands=(
    'build:build the workspace'
    'clean:remove workspace build/install/log'
    'source:print workspace source command'
    'cd:print workspace/package path'
    'pkg:package path helpers'
    'bag:ROS 2 bag helpers'
    'bridge:bridge helpers'
    'serial:serial compatibility group'
    'kill:process cleanup'
    'ws:workspace helpers'
    'help:show help'
  )
  bag_sub=('crop:crop bag' 'info:bag info' 'play:play bag' 'topics:list topics' 'help:show help')
  bridge_sub=('serial:serial bridge' 'help:show help')
  kill_sub=('ros:kill ROS 2 processes' 'help:show help')
  pkg_sub=('src:source path' 'share:share path' 'cd-src:cd command' 'help:show help')
  ws_sub=('status:workspace status' 'root:workspace root' 'build:build workspace' 'clean:clean workspace' 'help:show help')

  if (( CURRENT == 2 )); then
    _describe 'command' commands
    return
  fi

  case "$words[2]" in
    bag) _describe 'bag command' bag_sub ;;
    bridge) _describe 'bridge command' bridge_sub ;;
    kill) _describe 'kill command' kill_sub ;;
    pkg) _describe 'pkg command' pkg_sub ;;
    ws) _describe 'workspace command' ws_sub ;;
  esac
}

_farol "$@"
