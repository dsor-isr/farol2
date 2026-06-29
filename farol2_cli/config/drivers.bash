#!/usr/bin/env bash

# Package sets used by `farol drivers set <profile>`.
# Keep this file cheap to source: it is used by command completion.

FAROL_DRIVER_PROFILES=(
    magicelectric
)

FAROL_DRIVER_PROFILE_magicelectric=(
    farol2_drivers_bringup
    serial_lib
    serial_brushed_motor_wrapper
    vn310
    vn100
    can_thrusters
    can_instrumentation
    airmar200wx
    airmardx900
    ashtech_asio
)
