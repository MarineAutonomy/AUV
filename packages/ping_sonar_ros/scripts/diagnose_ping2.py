#!/usr/bin/env python3
"""Run inside auv: python3 /workspaces/mavlab/packages/ping_sonar_ros/scripts/diagnose_ping2.py"""
import os
import stat
import subprocess
import sys
import time

PORT = os.environ.get("PING2_PORT", "/dev/ping2")


def run(cmd):
    print(f"$ {cmd}")
    p = subprocess.run(cmd, shell=True, text=True, capture_output=True)
    out = (p.stdout or "") + (p.stderr or "")
    print(out.rstrip() or "(no output)")
    print()


print("=== device node ===")
run(f"ls -l {PORT} /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
run(f"udevadm info -q all -n {PORT} 2>/dev/null | grep -E 'DEVNAME|DEVLINKS|ID_SERIAL|ID_VENDOR|ID_MODEL|ID_USB'")
run("pgrep -af 'ping1d|ping360' || true")

print("=== serial without break (what initialize needs) ===")
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "ping_sonar_ros",
        "ping-python",
    ),
)
from brping.ping1d import Ping1D  # noqa: E402
import serial  # noqa: E402

if not os.path.exists(PORT):
    print(f"MISSING {PORT}")
    sys.exit(1)

mode = os.stat(PORT).st_mode
print(f"{PORT} mode={oct(stat.S_IMODE(mode))} link={os.path.islink(PORT)}")
if os.path.islink(PORT):
    print(f"  -> {os.path.realpath(PORT)}")

p = Ping1D()
p.iodev = serial.Serial(PORT, 115200, write_timeout=1.0, timeout=0.2)
print("opened without send_break")
time.sleep(0.6)
ok = p.initialize()
print(f"initialize() without break: {ok}")
if not ok:
    p.iodev.close()
    print("retrying library connect_serial (sends break + 'U')…")
    p2 = Ping1D()
    p2.connect_serial(PORT, 115200)
    time.sleep(0.8)
    print(f"initialize() after break: {p2.initialize()}")
else:
    info = p.get_firmware_version()
    print("firmware:", info)
    p.iodev.close()
