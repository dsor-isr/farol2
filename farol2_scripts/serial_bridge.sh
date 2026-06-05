serial_bridge() {
    if [ "$#" -ne 3 ]; then
        echo "Usage:"
        echo "  serial_bridge <real_port> <virtual_port> <baudrate>"
        echo
        echo "Example:"
        echo "  serial_bridge /dev/ttyACM0 /tmp/pico 115200"
        return 1
    fi

    local real_port="$1"
    local virtual_port="$2"
    local baudrate="$3"

    if [ ! -e "$real_port" ]; then
        echo "Error: real port does not exist: $real_port"
        return 1
    fi

    echo "Bridging serial:"
    echo "  real port:    $real_port"
    echo "  virtual port: $virtual_port"
    echo "  baudrate:     $baudrate"
    echo

    socat -x -v \
        PTY,link="$virtual_port",raw,echo=0 \
        "$real_port",b"$baudrate",raw,echo=0
}