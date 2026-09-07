#!/bin/bash
# List USB devices with serial IDs. Bound udevadm so a hung query cannot stall forever.
set -u

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev 2>/dev/null); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(timeout 2 udevadm info -q name -p "$syspath" 2>/dev/null)" || exit
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(timeout 2 udevadm info -q property --export -p "$syspath" 2>/dev/null)" || exit
        [[ -z "${ID_SERIAL:-}" ]] && exit
        echo "/dev/$devname - $ID_SERIAL"
    )
done
