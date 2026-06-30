#compdef farol

_farol() {
  local -a commands bag_sub bridge_sub drivers_sub kill_sub pkg_sub ws_sub driver_profiles
  commands=(
    'build:build the workspace or selected package'
    'clean:remove workspace build/install/log'
    'source:print workspace source command'
    'cd:print workspace/package path'
    'pkg:package path helpers'
    'bag:ROS 2 bag helpers'
    'bridge:bridge helpers'
    'serial:serial compatibility group'
    'drivers:driver package profiles'
    'kill:process cleanup'
    'ws:workspace helpers'
    'help:show help'
  )
  bag_sub=('crop:crop bag' 'info:bag info' 'play:play bag' 'topics:list topics' 'help:show help')
  bridge_sub=('serial:serial bridge' 'help:show help')
  drivers_sub=('disable:disable all drivers' 'enable:enable all drivers' 'set:set active driver profile' 'status:show driver state' 'profiles:list profiles' 'help:show help')
  driver_profiles=('all:enable all driver packages' 'magicelectric:Magic Electric driver packages')
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
    drivers)
      if (( CURRENT == 3 )); then
        _describe 'drivers command' drivers_sub
      elif [[ "$words[3]" == "set" && CURRENT == 4 ]]; then
        _describe 'driver profile' driver_profiles
      fi
      ;;
    kill) _describe 'kill command' kill_sub ;;
    pkg) _describe 'pkg command' pkg_sub ;;
    ws) _describe 'workspace command' ws_sub ;;
  esac
}

_farol "$@"
