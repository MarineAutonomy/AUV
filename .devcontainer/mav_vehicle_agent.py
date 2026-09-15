#!/usr/bin/env python3
"""MAV vehicle agent — runs on the vehicle (host network) for fast GUI ops.

ROS is sourced once at process start. MAV-GUI calls http://<jetson>:9123 instead of
SSH+docker-exec for sensors, devices, nav/control/missions.
"""
from __future__ import annotations

import base64
import json
import os
import re
import signal
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple
from urllib.parse import urlparse

# Same directory as this file (mounted at /workspaces/mavlab/.devcontainer/).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav_vehicle_agent_stacks as stacks  # noqa: E402

PORT = int(os.environ.get("MAV_AGENT_PORT", "9123"))
AUV_DIR = os.environ.get("MAV_AUV_DIR", "/workspaces/mavlab")
stacks.AUV_DIR = AUV_DIR

SENSOR_IDS = [
    "dvl",
    "sbg",
    "ping2",
    "ping360",
    "frontcam",
    "bottomcam",
    "modem",
    "bar30_ps",
    "sidescan",
]

# Match MAV-GUI backend sensorStartCommand (no redundant sros2 — env already loaded).
START_CMDS: Dict[str, str] = {
    "dvl": "ros2 launch dvl_a50 dvl_a50.launch.py ip_address:=192.168.194.95",
    "sbg": "ros2 launch sbg_driver sbg_device_launch.py",
    "ping2": "ros2 run ping_sonar_ros ping1d_node --ros-args -p port:=/dev/ping2",
    "ping360": "ros2 run ping360_sonar ping360.py --ros-args -p device:=/dev/ping360",
    "frontcam": (
        "ros2 run v4l2_camera v4l2_camera_node --ros-args "
        "-r __ns:=/front -p video_device:=/dev/frontcam -p image_size:=[640,480] "
        "-p framerate:=20.0 -p pixel_format:=YUYV -p output_encoding:=rgb8"
    ),
    "bottomcam": (
        "ros2 run v4l2_camera v4l2_camera_node --ros-args "
        "-r __ns:=/bottom -p video_device:=/dev/bottomcam -p image_size:=[640,480] "
        "-p framerate:=20.0 -p pixel_format:=YUYV -p output_encoding:=rgb8"
    ),
    "modem": "ros2 run modem_m64 modem_node --ros-args -r __ns:=/auv -p role:=b -p port:=/dev/modem",
    "bar30_ps": "ros2 run arduino_ps arduino_ps --ros-args -p port:=/dev/arduino_mega",
    "sidescan": "ros2 launch sidescan_ros2 sidescan.launch.py",
}

STOP_PATTERNS: Dict[str, str] = {
    "dvl": "dvl",
    "sbg": "sbg",
    "ping2": "ping1d_node",
    "ping360": "ping360",
    "frontcam": "video_device:=/dev/frontcam",
    "bottomcam": "video_device:=/dev/bottomcam",
    "modem": "modem",
    "bar30_ps": "arduino_ps|ms5837|bar30",
    "sidescan": "sidescan",
}

# Udev symlink basenames required before On. None = no USB symlink gate
# (sidescan / dvl require ethernet peers — see sensor_device_ready).
SENSOR_DEV_SYMLINKS: Dict[str, Optional[List[str]]] = {
    "dvl": None,
    "sbg": ["sbg"],
    "ping2": ["ping2"],
    "ping360": ["ping360"],
    "frontcam": ["frontcam"],
    "bottomcam": ["bottomcam"],
    "modem": ["modem"],
    "bar30_ps": ["arduino", "arduino_mega", "arduino_uno", "portenta"],
    "sidescan": None,
}

NOISE_RE = re.compile(
    r"pgrep|\bgrep\b|bash -lc|bash -ic|bash -c|docker exec|mav_sensor_agent|mav_vehicle_agent|sleep "
)

SYMLINK_RE = re.compile(r'SYMLINK\+="([^"]+)"')


def source_ros() -> None:
    cmd = (
        "bash -lc 'source /opt/ros/humble/setup.bash; "
        "source /home/mavlab/ros2_ws/install/setup.bash 2>/dev/null; "
        "source /workspaces/mavlab/code_ws/install/setup.bash 2>/dev/null; "
        "env -0'"
    )
    out = subprocess.check_output(cmd, shell=True)
    for entry in out.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        k, _, v = entry.partition(b"=")
        os.environ[k.decode("utf-8", "replace")] = v.decode("utf-8", "replace")


def _pgrep_lines(pattern: str) -> List[str]:
    try:
        out = subprocess.check_output(
            ["bash", "-lc", f"pgrep -u mavlab -af {pattern!r} || true"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return []
    return [
        ln
        for ln in out.splitlines()
        if ln.strip() and not NOISE_RE.search(ln)
    ]


def running(sensor_id: str) -> bool:
    if sensor_id in ("frontcam", "bottomcam"):
        # Only count a live v4l2_camera_node — not a shell wrapper whose cmdline
        # embeds the same strings (that caused green → red flicker after On).
        device = STOP_PATTERNS[sensor_id]
        ns = "__ns:=/front" if sensor_id == "frontcam" else "__ns:=/bottom"
        for ln in _pgrep_lines("v4l2_camera"):
            if "v4l2_camera_node" not in ln and "v4l2_camera " not in ln:
                continue
            if device in ln or ns in ln:
                return True
        return False

    pat = STOP_PATTERNS.get(sensor_id, sensor_id)
    # bar30 uses alternation — pgrep -f with | needs care; use grep.
    if "|" in pat:
        try:
            out = subprocess.check_output(
                [
                    "bash",
                    "-lc",
                    f"pgrep -u mavlab -af . 2>/dev/null | grep -E {pat!r} "
                    f"| grep -vE 'pgrep|grep|bash -lc|bash -ic|bash -c|docker exec|"
                    f"mav_sensor_agent|mav_vehicle_agent|sleep ' || true",
                ],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            return bool(out.strip())
        except Exception:
            return False
    return len(_pgrep_lines(pat)) > 0


def sensor_device_ready(sensor_id: str) -> bool:
    if sensor_id == "sidescan":
        # SS450 package needs both static sonar IPs reachable.
        return _ping_host("192.168.194.92") and _ping_host("192.168.194.93")
    if sensor_id == "dvl":
        return _ping_host("192.168.194.95")
    names = SENSOR_DEV_SYMLINKS.get(sensor_id)
    if names is None:
        return True
    return any(os.path.exists(f"/dev/{n}") for n in names)


def start_sensor(sensor_id: str) -> None:
    if running(sensor_id):
        # Idempotent: never spawn a second process (cams used to double-start when
        # status briefly showed red / Activate All raced with On).
        return
    if not sensor_device_ready(sensor_id):
        if sensor_id == "sidescan":
            raise RuntimeError(
                "sidescan: SS450 inactive — both 192.168.194.92 and 192.168.194.93 must be up"
            )
        if sensor_id == "dvl":
            raise RuntimeError("dvl: DVL inactive — 192.168.194.95 must be up")
        names = SENSOR_DEV_SYMLINKS.get(sensor_id) or []
        need = ", ".join(f"/dev/{n}" for n in names)
        raise RuntimeError(f"{sensor_id}: device symlink not active ({need})")
    cmd = START_CMDS[sensor_id]

    def _spawn() -> None:
        # Brief settle for USB cams without embedding `sleep` in the process cmdline
        # (that used to trip status probes and make the GUI flip On→Off).
        if sensor_id in ("frontcam", "bottomcam"):
            time.sleep(0.4)
        log_path = f"/tmp/mav_{sensor_id}.log"
        try:
            log_f = open(log_path, "ab", buffering=0)
        except Exception:
            log_f = subprocess.DEVNULL
        subprocess.Popen(
            cmd,
            shell=True,
            executable="/bin/bash",
            stdout=log_f,
            stderr=log_f,
            start_new_session=True,
            env=os.environ.copy(),
        )

    if sensor_id in ("frontcam", "bottomcam"):
        import threading

        threading.Thread(target=_spawn, daemon=True).start()
    else:
        _spawn()


def stop_sensor(sensor_id: str) -> None:
    pat = STOP_PATTERNS[sensor_id]
    subprocess.run(
        ["bash", "-lc", f"pkill -u mavlab -9 -f {pat!r} >/dev/null 2>&1 || true"],
        check=False,
    )


def activate_all() -> None:
    for sid in SENSOR_IDS:
        if running(sid) or not sensor_device_ready(sid):
            continue
        try:
            start_sensor(sid)
        except RuntimeError:
            continue


def deactivate_all() -> None:
    for sid in SENSOR_IDS:
        if running(sid):
            stop_sensor(sid)


def list_usb_devices() -> List[str]:
    """Same output as repo usb_devices.sh — prefer host udev via nsenter.

    Inside the privileged agent container, `udevadm` often lacks ID_SERIAL. mavlab can
    `sudo -n nsenter` into PID 1 (same view as SSH → ./usb_devices.sh).
    """
    script = os.path.join(AUV_DIR, "usb_devices.sh")
    if not os.path.isfile(script):
        raise FileNotFoundError(f"missing {script}")
    with open(script, "r", encoding="utf-8", errors="replace") as f:
        script_body = f.read()

    attempts: List[tuple] = []
    nsenter = "/usr/bin/nsenter" if os.path.exists("/usr/bin/nsenter") else "/nsenter"
    if os.path.exists(nsenter):
        attempts.append(
            (
                ["sudo", "-n", nsenter, "-t", "1", "-m", "-u", "-i", "-n", "--", "bash", "-s"],
                script_body,
            )
        )
        attempts.append(
            (
                [nsenter, "-t", "1", "-m", "-u", "-i", "-n", "--", "bash", "-s"],
                script_body,
            )
        )
    attempts.append((["bash", script], None))

    last_err = "usb_devices.sh failed"
    for cmd, stdin in attempts:
        try:
            r = subprocess.run(
                cmd,
                input=stdin,
                cwd=AUV_DIR if stdin is None else None,
                capture_output=True,
                text=True,
                timeout=12,
                check=False,
            )
        except FileNotFoundError:
            continue
        except Exception as e:
            last_err = str(e)
            continue
        lines = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
        if r.returncode == 0 and lines:
            return lines
        if r.returncode == 0 and stdin is None:
            # Local script can succeed with empty ID_SERIAL filter — keep trying better paths.
            last_err = "no devices (local udev incomplete?)"
            continue
        if r.returncode == 0:
            return lines
        last_err = (r.stderr or r.stdout or last_err).strip()
    # Last resort: local empty list is still a valid answer if nothing is plugged in.
    try:
        r = subprocess.run(
            ["bash", script],
            cwd=AUV_DIR,
            capture_output=True,
            text=True,
            timeout=12,
            check=False,
        )
        if r.returncode == 0:
            return [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    except Exception as e:
        last_err = str(e)
    raise RuntimeError(last_err)


def udev_mappings() -> List[dict]:
    """Match MAV-GUI backend udevMapRemoteScript output shape."""
    rules_file = os.path.join(AUV_DIR, "99-usb-serial.rules")
    names: List[str] = []
    if os.path.isfile(rules_file):
        with open(rules_file, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = SYMLINK_RE.search(line)
                if m:
                    names.append(m.group(1))
    mappings: List[dict] = []
    for name in names:
        path = f"/dev/{name}"
        if os.path.exists(path):
            real: Optional[str] = None
            try:
                real = os.path.realpath(path)
            except OSError:
                real = None
            mappings.append(
                {
                    "symlink": path,
                    "realDevice": real,
                    "status": "active",
                }
            )
        else:
            mappings.append(
                {
                    "symlink": path,
                    "realDevice": None,
                    "status": "not_found",
                }
            )
    return mappings


# Static LAN peers for Sensors that are not USB/udev (ping from vehicle).
ETHERNET_DEVICES = [
    {
        "id": "ss450",
        "label": "SS450",
        "endpoints": [
            {"name": "sonar_a", "host": "192.168.194.92"},
            {"name": "sonar_b", "host": "192.168.194.93"},
        ],
    },
    {
        "id": "dvl",
        "label": "DVL",
        "endpoints": [
            {"name": "dvl", "host": "192.168.194.95"},
        ],
    },
    {
        "id": "fs450",
        "label": "FS450",
        "endpoints": [
            {"name": "fs450", "host": "192.168.194.90"},
        ],
    },
]


def _ping_host(host: str, timeout_s: float = 1.5) -> bool:
    """ICMP reachability from the vehicle.

    The agent image has no `ping` — enter the host mount+net namespaces to use
    /usr/bin/ping (same trick as USB listing).
    """
    wait = str(max(1, int(timeout_s)))
    ping_args = ["-c", "1", "-W", wait, host]
    attempts: List[List[str]] = [
        ["ping", *ping_args],
        ["/usr/bin/ping", *ping_args],
    ]
    nsenter = "/usr/bin/nsenter" if os.path.exists("/usr/bin/nsenter") else "/nsenter"
    if os.path.exists(nsenter):
        host_ping = [nsenter, "-t", "1", "-m", "-n", "--", "/usr/bin/ping", *ping_args]
        attempts.append(["sudo", "-n", *host_ping])
        attempts.append(host_ping)
    for cmd in attempts:
        try:
            r = subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout_s + 1.0,
                check=False,
            )
            if r.returncode == 0:
                return True
        except Exception:
            continue
    return False


def ethernet_devices() -> List[dict]:
    """ACTIVE/INACTIVE for DVL + SS450 static IPs (both SS450 sonars required for active)."""
    import concurrent.futures

    hosts: List[str] = []
    for dev in ETHERNET_DEVICES:
        for ep in dev["endpoints"]:
            hosts.append(ep["host"])
    reach: Dict[str, bool] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(hosts) or 1) as pool:
        futs = {pool.submit(_ping_host, h): h for h in hosts}
        for fut in concurrent.futures.as_completed(futs):
            reach[futs[fut]] = bool(fut.result())

    out: List[dict] = []
    for dev in ETHERNET_DEVICES:
        endpoints = []
        for ep in dev["endpoints"]:
            ok = reach.get(ep["host"], False)
            endpoints.append(
                {
                    "name": ep["name"],
                    "host": ep["host"],
                    "status": "active" if ok else "inactive",
                }
            )
        ups = sum(1 for e in endpoints if e["status"] == "active")
        if ups == len(endpoints) and ups > 0:
            status = "active"
        elif ups == 0:
            status = "inactive"
        else:
            status = "partial"
        out.append(
            {
                "id": dev["id"],
                "label": dev["label"],
                "status": status,
                "endpoints": endpoints,
            }
        )
    return out


def _nsenter_bash_cmds() -> List[List[str]]:
    nsenter = "/usr/bin/nsenter" if os.path.exists("/usr/bin/nsenter") else "/nsenter"
    if not os.path.exists(nsenter):
        return []
    return [
        ["sudo", "-n", nsenter, "-t", "1", "-m", "-u", "-i", "-n", "--", "bash", "-s"],
        [nsenter, "-t", "1", "-m", "-u", "-i", "-n", "--", "bash", "-s"],
    ]


def run_on_host(script: str, timeout: float = 30) -> Tuple[int, str, str]:
    """Run a bash script in the host namespaces (must affect host /etc/udev, not the container)."""
    last_err = "nsenter unavailable"
    for cmd in _nsenter_bash_cmds():
        try:
            r = subprocess.run(
                cmd,
                input=script,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except FileNotFoundError:
            continue
        except Exception as e:
            last_err = str(e)
            continue
        if r.returncode == 0:
            return 0, r.stdout or "", r.stderr or ""
        last_err = (r.stderr or r.stdout or f"exit {r.returncode}").strip()
    raise RuntimeError(last_err)


def apply_udev_rules() -> str:
    """Install repo 99-usb-serial.rules on the Jetson host and reload udev."""

    rules_path = os.path.join(AUV_DIR, "99-usb-serial.rules")
    if not os.path.isfile(rules_path):
        raise FileNotFoundError(f"missing {rules_path}")
    raw = open(rules_path, "rb").read()
    b64 = base64.b64encode(raw).decode("ascii")
    host_script = (
        "set -euo pipefail\n"
        f"printf '%s' '{b64}' | base64 -d > /etc/udev/rules.d/99-usb-serial.rules\n"
        "udevadm control --reload-rules\n"
        "udevadm trigger\n"
        "echo 'udev rules applied'\n"
    )
    code, stdout, stderr = run_on_host(host_script, timeout=25)
    if code != 0:
        raise RuntimeError(stderr or stdout or "udev apply failed")
    # Symlinks can appear a moment after trigger; wait so the returned map is settled.
    time.sleep(0.9)
    return (stdout or "").strip() or "udev rules applied"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        pass

    def _json(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/health":
            self._json(200, {"ok": True, "service": "mav_vehicle_agent"})
            return
        if path == "/sensors/status":
            self._json(200, {"status": {sid: running(sid) for sid in SENSOR_IDS}})
            return
        m = re.match(r"^/sensors/status/([a-z0-9_]+)$", path)
        if m:
            sid = m.group(1)
            if sid not in START_CMDS:
                self._json(400, {"error": "unknown sensor id"})
                return
            self._json(200, {"id": sid, "running": running(sid)})
            return
        if path == "/devices/usb":
            try:
                devices = list_usb_devices()
                self._json(200, {"ok": True, "code": 0, "devices": devices, "via": "vehicle-agent"})
            except Exception as e:
                self._json(500, {"ok": False, "code": 1, "error": str(e), "devices": []})
            return
        if path == "/devices/udev-map":
            try:
                mappings = udev_mappings()
                self._json(
                    200,
                    {"ok": True, "code": 0, "mappings": mappings, "via": "vehicle-agent"},
                )
            except Exception as e:
                self._json(500, {"ok": False, "code": 1, "error": str(e), "mappings": []})
            return
        if path == "/devices/ethernet":
            try:
                devices = ethernet_devices()
                self._json(
                    200,
                    {"ok": True, "code": 0, "devices": devices, "via": "vehicle-agent"},
                )
            except Exception as e:
                self._json(500, {"ok": False, "code": 1, "error": str(e), "devices": []})
            return
        handled = stacks.handle_get(path)
        if handled is not None:
            code, obj = handled
            self._json(code, obj)
            return
        self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/sensors/activate":
            activate_all()
            self._json(200, {"ok": True, "action": "activate"})
            return
        if path == "/sensors/deactivate":
            deactivate_all()
            self._json(200, {"ok": True, "action": "deactivate"})
            return
        if path == "/devices/udev":
            try:
                stdout = apply_udev_rules()
                self._json(
                    200,
                    {
                        "ok": True,
                        "code": 0,
                        "stdout": stdout,
                        "stderr": "",
                        "mappings": udev_mappings(),
                        "via": "vehicle-agent",
                    },
                )
            except Exception as e:
                self._json(
                    500,
                    {
                        "ok": False,
                        "code": 1,
                        "error": str(e),
                        "stdout": "",
                        "stderr": str(e),
                        "mappings": [],
                    },
                )
            return
        m = re.match(r"^/sensors/(start|stop)/([a-z0-9_]+)$", path)
        if m:
            action, sid = m.group(1), m.group(2)
            if sid not in START_CMDS:
                self._json(400, {"error": "unknown sensor id"})
                return
            try:
                if action == "start":
                    start_sensor(sid)
                else:
                    stop_sensor(sid)
                self._json(200, {"ok": True, "code": 0, "id": sid, "action": action, "via": "agent"})
            except RuntimeError as e:
                self._json(
                    409,
                    {
                        "ok": False,
                        "code": 1,
                        "id": sid,
                        "action": action,
                        "error": str(e),
                        "stdout": "",
                        "stderr": str(e),
                        "via": "agent",
                    },
                )
            except Exception as e:
                self._json(500, {"ok": False, "code": 1, "error": str(e)})
            return

        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except Exception:
            body = {}
        handled = stacks.handle_post(path, body if isinstance(body, dict) else {})
        if handled is not None:
            code, obj = handled
            self._json(code, obj)
            return
        self._json(404, {"error": "not found"})

    def do_PUT(self) -> None:
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except Exception:
            body = {}
        handled = stacks.handle_put(path, body if isinstance(body, dict) else {})
        if handled is not None:
            code, obj = handled
            self._json(code, obj)
            return
        self._json(404, {"error": "not found"})

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        handled = stacks.handle_delete(path)
        if handled is not None:
            code, obj = handled
            self._json(code, obj)
            return
        self._json(404, {"error": "not found"})


def main() -> None:
    print("[mav-vehicle-agent] sourcing ROS once…", flush=True)
    source_ros()
    print(f"[mav-vehicle-agent] listening on 0.0.0.0:{PORT} (auv={AUV_DIR})", flush=True)
    ThreadingHTTPServer.allow_reuse_address = True
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)

    def _shutdown(*_args: object) -> None:
        threading_shutdown = getattr(httpd, "shutdown", None)
        if callable(threading_shutdown):
            import threading

            threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
