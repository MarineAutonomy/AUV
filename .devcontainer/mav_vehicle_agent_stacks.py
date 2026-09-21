"""Nav / control / missions / rosbag helpers for mav_vehicle_agent (local HTTP, no SSH)."""
from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

AUV_DIR = os.environ.get("MAV_AUV_DIR", "/workspaces/mavlab")

NUM_RE = re.compile(r"^\s*([a-z_]+):\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$")
ARR_RE = re.compile(r"^\s*([a-z_]+):\s*\[(.*)\]\s*$")
SENSOR_RE = re.compile(r"^\s*-\s*sensor_type:\s*([A-Z0-9_]+)\s*$")
NOISE_RE = re.compile(
    r"pgrep|\bgrep\b|bash -lc|bash -ic|docker exec|mav_sensor_agent|mav_vehicle_agent"
)

CONTROL_2D_KEYS = [
    "kp_u",
    "ki_u",
    "kd_u",
    "kp_v",
    "ki_v",
    "kd_v",
    "kp_yaw",
    "ki_yaw",
    "kd_yaw",
]
CONTROL_3D_KEYS = CONTROL_2D_KEYS + [
    "kp_w",
    "ki_w",
    "kd_w",
    "kp_roll",
    "ki_roll",
    "kd_roll",
    "kp_pitch",
    "ki_pitch",
    "kd_pitch",
]

MISSION_META: Dict[str, Dict[str, Any]] = {
    "2d_ilos": {
        "pattern": "point_tracking_mission_2d_ilos",
        "pkg": "auv_2d_missions",
        "exe": "point_tracking_mission_2d_ilos",
        "yaml": "code_ws/src/auv_2d_missions/config/point_tracking/point_tracking_ilos.yaml",
        "scalars": [
            "u_cruise",
            "u_min",
            "u_max",
            "lookahead",
            "ilos_ki",
            "ilos_int_limit",
            "acceptance_radius",
            "switch_alongtrack_margin",
        ],
        "waypoints": ["waypoints_x", "waypoints_y"],
        "dims": 2,
        "start": "waypoints",
    },
    "2d_los": {
        "pattern": "point_tracking_mission_2d_los",
        "pkg": "auv_2d_missions",
        "exe": "point_tracking_mission_2d_los",
        "yaml": "code_ws/src/auv_2d_missions/config/point_tracking/point_tracking_los.yaml",
        "scalars": [
            "u_cruise",
            "u_min",
            "u_max",
            "lookahead",
            "acceptance_radius",
            "switch_alongtrack_margin",
        ],
        "waypoints": ["waypoints_x", "waypoints_y"],
        "dims": 2,
        "start": "waypoints",
    },
    "3d_ilos": {
        "pattern": "point_tracking_mission_3d_ilos",
        "pkg": "auv_3d_missions",
        "exe": "point_tracking_mission_3d_ilos",
        "yaml": "code_ws/src/auv_3d_missions/config/point_tracking/point_tracking_ilos_3d.yaml",
        "scalars": [
            "u_cruise",
            "u_min",
            "u_max",
            "lookahead_xy",
            "lookahead_z",
            "ilos_ki_xy",
            "ilos_ki_z",
            "ilos_int_limit_xy",
            "ilos_int_limit_z",
            "acceptance_radius",
            "switch_alongtrack_margin",
            "w_gain",
            "w_max",
        ],
        "waypoints": ["waypoints_x", "waypoints_y", "waypoints_z"],
        "dims": 3,
        "start": "waypoints",
    },
    "3d_los": {
        "pattern": "point_tracking_mission_3d_los",
        "pkg": "auv_3d_missions",
        "exe": "point_tracking_mission_3d_los",
        "yaml": "code_ws/src/auv_3d_missions/config/point_tracking/point_tracking_los_3d.yaml",
        "scalars": [
            "u_cruise",
            "u_min",
            "u_max",
            "lookahead_xy",
            "lookahead_z",
            "acceptance_radius",
            "switch_alongtrack_margin",
            "w_gain",
            "w_max",
        ],
        "waypoints": ["waypoints_x", "waypoints_y", "waypoints_z"],
        "dims": 3,
        "start": "waypoints",
    },
    "3d_depth": {
        "pattern": "depth_control_mission_3d",
        "pkg": "auv_3d_missions",
        "exe": "depth_control_mission_3d",
        "yaml": "code_ws/src/auv_3d_missions/config/depth_control/depth_control_3d.yaml",
        "scalars": [
            "target_depth",
            "w_gain",
            "w_max",
            "lookahead_z",
            "depth_tolerance",
        ],
        "waypoints": [],
        "dims": 3,
        "start": "depth",
    },
    "3d_station": {
        "pattern": "station_keeping_mission_3d",
        "pkg": "auv_3d_missions",
        "exe": "station_keeping_mission_3d",
        "yaml": "code_ws/src/auv_3d_missions/config/station_keeping/station_keeping_3d.yaml",
        "scalars": [
            "target_x",
            "target_y",
            "target_z",
            "target_roll_deg",
            "target_pitch_deg",
            "target_yaw_deg",
            "kp_xy",
            "u_max",
            "v_max",
            "w_gain",
            "w_max",
            "lookahead_z",
            "pos_tolerance",
        ],
        "waypoints": [],
        "dims": 3,
        "start": "station",
    },
    "3d_dwell": {
        "pattern": "dwell_mission_3d",
        "pkg": "auv_3d_missions",
        "exe": "dwell_mission_3d",
        "yaml": "code_ws/src/auv_3d_missions/config/dwell/dwell_3d.yaml",
        "scalars": [
            "target_x",
            "target_y",
            "target_z",
            "target_pitch_deg",
            "target_yaw_deg",
            "initial_hold_s",
            "cycles",
            "cycle_hold_s",
            "roll_hold_s",
            "kp_xy",
            "u_max",
            "v_max",
            "w_gain",
            "w_max",
            "pos_tolerance",
        ],
        "waypoints": [],
        "dims": 3,
        "start": "dwell",
    },
}

_bag_active_run: Optional[str] = None
_rosbag_active_name: Optional[str] = None
_rosbag_active_pids: List[int] = []


def _path(*parts: str) -> str:
    return os.path.join(AUV_DIR, *parts)


def process_running(pattern: str) -> bool:
    # Alternation and regex dots need grep -E; plain pgrep -f treats | / .* oddly.
    if "|" in pattern or ".*" in pattern:
        try:
            out = subprocess.check_output(
                [
                    "bash",
                    "-lc",
                    f"pgrep -u mavlab -af . 2>/dev/null | grep -E {pattern!r} "
                    f"| grep -vE 'pgrep|grep|bash -lc|bash -ic|docker exec|"
                    f"mav_sensor_agent|mav_vehicle_agent' || true",
                ],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            return bool(out.strip())
        except Exception:
            return False
    try:
        out = subprocess.check_output(
            ["bash", "-lc", f"pgrep -u mavlab -af {pattern!r} || true"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return False
    return any(ln.strip() and not NOISE_RE.search(ln) for ln in out.splitlines())


def stop_pattern(pattern: str) -> None:
    subprocess.run(
        ["bash", "-lc", f"pkill -u mavlab -9 -f {pattern!r} >/dev/null 2>&1 || true"],
        check=False,
    )


def start_detached(cmd: str) -> None:
    subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=os.environ.copy(),
    )


def wait_running(pattern: str, attempts: int = 8, interval_s: float = 0.4) -> bool:
    for _ in range(attempts):
        if process_running(pattern):
            return True
        time.sleep(interval_s)
    return False


def read_scalar_yaml(path: str, keys: Sequence[str]) -> Dict[str, float]:
    keyset = set(keys)
    vals: Dict[str, float] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = NUM_RE.match(line.rstrip("\n"))
            if m and m.group(1) in keyset:
                vals[m.group(1)] = float(m.group(2))
    return vals


def write_scalar_yaml(path: str, updates: Dict[str, float]) -> None:
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    out: List[str] = []
    seen = set()
    for line in lines:
        m = NUM_RE.match(line.rstrip("\n"))
        if m and m.group(1) in updates:
            k = m.group(1)
            indent = line[: len(line) - len(line.lstrip())]
            out.append(f"{indent}{k}: {float(updates[k]):.6f}\n")
            seen.add(k)
        else:
            out.append(line)
    missing = [k for k in updates if k not in seen]
    if missing:
        raise RuntimeError(f"missing keys in {path}: {', '.join(missing)}")
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(out)


def read_mission_yaml(path: str, scalars: Sequence[str], waypoints: Sequence[str]) -> Dict[str, Any]:
    skeys = set(scalars)
    wkeys = set(waypoints)
    vals: Dict[str, Any] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.rstrip("\n")
            m = NUM_RE.match(s)
            if m and m.group(1) in skeys:
                vals[m.group(1)] = float(m.group(2))
                continue
            a = ARR_RE.match(s)
            if a and a.group(1) in wkeys:
                raw = a.group(2).strip()
                vals[a.group(1)] = (
                    [] if raw == "" else [float(x.strip()) for x in raw.split(",") if x.strip()]
                )
    return vals


def control_pattern(is3d: bool) -> str:
    return (
        "controller_3d|thruster_allocator_3d|control_3d.launch.py"
        if is3d
        else "controller_2d|thruster_allocator_2d|control_2d.launch.py"
    )


def control_yaml(is3d: bool) -> str:
    return _path(
        "code_ws/src",
        "auv_3d_control" if is3d else "auv_2d_control",
        "config",
        "control_3d_default.yaml" if is3d else "control_2d_default.yaml",
    )


def control_status(is3d: bool) -> Dict[str, Any]:
    return {"running": process_running(control_pattern(is3d)), "via": "vehicle-agent"}


def control_start(is3d: bool) -> Dict[str, Any]:
    if process_running(control_pattern(is3d)):
        return {
            "code": 0,
            "skipped": True,
            "stdout": f"control{'3d' if is3d else '2d'}: already running (skipped start)\n",
            "via": "vehicle-agent",
        }
    pkg = "auv_3d_control" if is3d else "auv_2d_control"
    launch = "control_3d.launch.py" if is3d else "control_2d.launch.py"
    start_detached(f"ros2 launch {pkg} {launch}")
    return {"code": 0, "via": "vehicle-agent"}


def control_stop(is3d: bool) -> Dict[str, Any]:
    stop_pattern(control_pattern(is3d))
    return {"ok": True, "code": 0, "stdout": "", "stderr": "", "via": "vehicle-agent"}


def control_pid_get(is3d: bool) -> Dict[str, Any]:
    keys = CONTROL_3D_KEYS if is3d else CONTROL_2D_KEYS
    return read_scalar_yaml(control_yaml(is3d), keys)


def control_pid_set(is3d: bool, body: Dict[str, Any]) -> Dict[str, Any]:
    keys = CONTROL_3D_KEYS if is3d else CONTROL_2D_KEYS
    updates = {k: float(body[k]) for k in keys}
    write_scalar_yaml(control_yaml(is3d), updates)
    return {"code": 0, "saved": True, "via": "vehicle-agent"}


def mission_status(kind: str) -> Dict[str, Any]:
    meta = MISSION_META[kind]
    return {"running": process_running(meta["pattern"]), "via": "vehicle-agent"}


def mission_params_get(kind: str) -> Dict[str, Any]:
    meta = MISSION_META[kind]
    return read_mission_yaml(_path(meta["yaml"]), meta["scalars"], meta["waypoints"])


def mission_params_set(kind: str, body: Dict[str, Any]) -> Dict[str, Any]:
    meta = MISSION_META[kind]
    updates = {k: float(body[k]) for k in meta["scalars"]}
    write_scalar_yaml(_path(meta["yaml"]), updates)
    return {"code": 0, "saved": True, "via": "vehicle-agent"}


def mission_start(kind: str, body: Dict[str, Any]) -> Dict[str, Any]:
    meta = MISSION_META[kind]
    if process_running(meta["pattern"]):
        return {
            "code": 0,
            "skipped": True,
            "stdout": f"{kind}: already running (skipped start)\n",
            "via": "vehicle-agent",
        }
    params = _path(meta["yaml"])
    if meta.get("start") == "depth":
        depth = body.get("depth", body.get("target_depth"))
        if depth is None:
            raise ValueError("depth (target depth in meters) required")
        d = float(depth)
        cmd = (
            f"ros2 run {meta['pkg']} {meta['exe']} --ros-args "
            f"--params-file {params!r} "
            f"-p target_depth:={d:.6f}"
        )
        start_detached(cmd)
        return {"code": 0, "via": "vehicle-agent"}

    if meta.get("start") == "station":
        try:
            x = float(body.get("x", body.get("target_x")))
            y = float(body.get("y", body.get("target_y")))
            z = float(body.get("z", body.get("target_z")))
            roll = float(body.get("roll", body.get("target_roll_deg", 0.0)))
            pitch = float(body.get("pitch", body.get("target_pitch_deg", 0.0)))
            yaw = float(body.get("yaw", body.get("target_yaw_deg", 0.0)))
        except (TypeError, ValueError) as e:
            raise ValueError("station keeping requires numeric x, y, z, roll, pitch, yaw") from e
        cmd = (
            f"ros2 run {meta['pkg']} {meta['exe']} --ros-args "
            f"--params-file {params!r} "
            f"-p target_x:={x:.6f} -p target_y:={y:.6f} -p target_z:={z:.6f} "
            f"-p target_roll_deg:={roll:.6f} -p target_pitch_deg:={pitch:.6f} "
            f"-p target_yaw_deg:={yaw:.6f}"
        )
        start_detached(cmd)
        return {"code": 0, "via": "vehicle-agent"}

    if meta.get("start") == "dwell":
        try:
            x = float(body.get("x", body.get("target_x")))
            y = float(body.get("y", body.get("target_y")))
            z = float(body.get("z", body.get("target_z")))
            pitch = float(body.get("pitch", body.get("target_pitch_deg", 0.0)))
            yaw = float(body.get("yaw", body.get("target_yaw_deg", 0.0)))
            initial_hold_s = float(body.get("initial_hold_s", 10.0))
            cycles = int(body.get("cycles", 1))
            cycle_hold_s = float(body.get("cycle_hold_s", 10.0))
            roll_hold_s = float(body.get("roll_hold_s", 4.0))
        except (TypeError, ValueError) as e:
            raise ValueError(
                "dwell requires numeric x, y, z, pitch, yaw, initial_hold_s, cycles, cycle_hold_s, roll_hold_s"
            ) from e
        if cycles < 1:
            raise ValueError("dwell cycles must be >= 1")
        cmd = (
            f"ros2 run {meta['pkg']} {meta['exe']} --ros-args "
            f"--params-file {params!r} "
            f"-p target_x:={x:.6f} -p target_y:={y:.6f} -p target_z:={z:.6f} "
            f"-p target_pitch_deg:={pitch:.6f} -p target_yaw_deg:={yaw:.6f} "
            f"-p initial_hold_s:={initial_hold_s:.6f} -p cycles:={cycles} "
            f"-p cycle_hold_s:={cycle_hold_s:.6f} -p roll_hold_s:={roll_hold_s:.6f}"
        )
        start_detached(cmd)
        return {"code": 0, "via": "vehicle-agent"}

    points = body.get("points") if isinstance(body, dict) else body
    if not isinstance(points, list):
        points = []
    if len(points) < 2:
        raise ValueError("need at least 2 points")
    xs = ", ".join(f"{float(p['x']):.3f}" for p in points)
    ys = ", ".join(f"{float(p['y']):.3f}" for p in points)
    cmd = (
        f"ros2 run {meta['pkg']} {meta['exe']} --ros-args "
        f"--params-file {params!r} "
        f'-p waypoints_x:="[{xs}]" -p waypoints_y:="[{ys}]"'
    )
    if meta["dims"] == 3:
        zs = ", ".join(f"{float(p['z']):.3f}" for p in points)
        cmd += f' -p waypoints_z:="[{zs}]"'
    start_detached(cmd)
    return {"code": 0, "via": "vehicle-agent"}


def mission_stop(kind: str) -> Dict[str, Any]:
    stop_pattern(MISSION_META[kind]["pattern"])
    return {"ok": True, "code": 0, "stdout": "", "stderr": "", "via": "vehicle-agent"}


def _nav_cmd(use_sim_time: bool) -> str:
    candidates = [
        "/workspaces/mavlab/code_ws/src/auv_navigation/config/vessel_data.yml",
        "/workspaces/mavlab/code_ws/src/auv_navigation/config/vessel_data.example.yml",
        "/home/mavlab/AUV/code_ws/src/auv_navigation/config/vessel_data.yml",
        "/home/mavlab/AUV/code_ws/src/auv_navigation/config/vessel_data.example.yml",
    ]
    sim = " -p use_sim_time:=true" if use_sim_time else ""
    vfile = ""
    for p in candidates:
        if os.path.isfile(p):
            vfile = p
            break
    if vfile:
        return f"ros2 run auv_navigation navigation_node --ros-args -p vessel_data_file:={vfile}{sim}"
    return f"ros2 run auv_navigation navigation_node --ros-args{sim}"


def nav_status() -> Dict[str, Any]:
    global _bag_active_run
    running = process_running("navigation_node")
    bag_playback = bool(_bag_active_run)
    if _bag_active_run and not process_running("ros2 bag play"):
        _bag_active_run = None
        bag_playback = False
    return {
        "navigationRunning": running,
        "bagPlayback": bag_playback,
        "bagRun": _bag_active_run,
        "statusTimeout": False,
        "via": "vehicle-agent",
    }


def nav_start(mode: str, bag_name: Optional[str] = None) -> Dict[str, Any]:
    global _bag_active_run
    # ENU→NED is done in-process by navigation_node (publishes /imu/data/ned).
    # Do not rely on a separate imu_enu_to_ned process (entry-point often missing
    # on vehicle installs; process probes were false-positive).
    if process_running("navigation_node"):
        return {
            "code": 0,
            "skipped": True,
            "mode": mode,
            "stdout": "navigation: already running (skipped start)\n",
            "via": "vehicle-agent",
        }
    if mode == "bag":
        bn = (bag_name or "").strip()
        if not re.fullmatch(r"run\d+", bn or ""):
            raise ValueError("bagName (e.g. run3) required for bag mode")
        bag_path = _path("rosbags", bn)
        start_detached(f"ros2 bag play {bag_path!r} --clock")
        _bag_active_run = bn
        start_detached(_nav_cmd(True))
    else:
        start_detached(_nav_cmd(False))
    if not wait_running("navigation_node"):
        raise RuntimeError(
            "navigation_node did not stay running after start "
            "(check container logs/terminal and vessel config path)."
        )
    return {"code": 0, "mode": mode, "via": "vehicle-agent"}


def nav_stop() -> Dict[str, Any]:
    global _bag_active_run
    stop_pattern("ros2 bag play")
    stop_pattern("navigation_node")
    # Legacy separate converter (if an older agent left one running).
    stop_pattern("imu_enu_to_ned")
    _bag_active_run = None
    return {"code": 0, "stopped": True, "via": "vehicle-agent"}


def vessel_yaml_path() -> str:
    return _path("code_ws/src/auv_navigation/config/vessel_data.example.yml")


def ekf_params_get() -> Dict[str, Any]:
    path = vessel_yaml_path()
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    top: Dict[str, float] = {}
    for line in lines:
        m = NUM_RE.match(line.rstrip("\n"))
        if m and m.group(1) in ("time_step", "pool_depth", "dvl_depth_from_surface"):
            top[m.group(1)] = float(m.group(2))

    sensor_ranges: Dict[str, List[int]] = {}
    order: List[str] = []
    for i, line in enumerate(lines):
        m = SENSOR_RE.match(line.rstrip("\n"))
        if m:
            st = m.group(1)
            sensor_ranges[st] = [i, len(lines)]
            order.append(st)
    for i in range(len(order) - 1):
        sensor_ranges[order[i]][1] = sensor_ranges[order[i + 1]][0]

    def read_sensor(st: str) -> Dict[str, Any]:
        if st not in sensor_ranges:
            return {}
        s, e = sensor_ranges[st]
        out: Dict[str, Any] = {}
        for i in range(s + 1, e):
            ln = lines[i].rstrip("\n")
            m = NUM_RE.match(ln)
            if m:
                out[m.group(1)] = float(m.group(2))
                continue
            a = ARR_RE.match(ln)
            if a and a.group(1) in ("sensor_location", "sensor_orientation"):
                raw = a.group(2).strip()
                out[a.group(1)] = (
                    [] if raw == "" else [float(x.strip()) for x in raw.split(",") if x.strip()]
                )
        return out

    imu = read_sensor("IMU") or read_sensor("IMU_SBG")
    depth = read_sensor("ARDUINO_DEPTH")
    dvl = read_sensor("DVL")
    dvl_dr = read_sensor("DVL_DR")
    ping = read_sensor("PING_RANGE")
    z3 = [0.0, 0.0, 0.0]
    return {
        "time_step": float(top.get("time_step", 0.1)),
        "pool_depth": float(top.get("pool_depth", 3.0)),
        "dvl_depth_from_surface": float(top.get("dvl_depth_from_surface", 0.45)),
        "imu_default_gyro_variance": float(imu.get("default_gyro_variance", 0.001)),
        "imu_default_accel_variance": float(imu.get("default_accel_variance", 0.005)),
        "arduino_depth_default_variance": float(depth.get("default_variance", 0.005)),
        "dvl_default_velocity_variance": float(dvl.get("default_velocity_variance", 0.0005)),
        "dvl_dr_min_position_variance": float(dvl_dr.get("min_position_variance", 0.005)),
        "dvl_dr_position_variance_scale": float(dvl_dr.get("position_variance_scale", 1.0)),
        "dvl_dr_drift_rate": float(dvl_dr.get("dr_drift_rate", 0.001)),
        "ping_range_default_variance": float(ping.get("default_variance", 0.001)),
        "imu_sbg_sensor_location": [float(x) for x in (imu.get("sensor_location") or z3)],
        "imu_sbg_sensor_orientation": [float(x) for x in (imu.get("sensor_orientation") or z3)],
        "arduino_depth_sensor_location": [float(x) for x in (depth.get("sensor_location") or z3)],
        "arduino_depth_sensor_orientation": [
            float(x) for x in (depth.get("sensor_orientation") or z3)
        ],
        "dvl_sensor_location": [float(x) for x in (dvl.get("sensor_location") or z3)],
        "dvl_sensor_orientation": [float(x) for x in (dvl.get("sensor_orientation") or z3)],
        "dvl_dr_sensor_location": [float(x) for x in (dvl_dr.get("sensor_location") or z3)],
        "dvl_dr_sensor_orientation": [float(x) for x in (dvl_dr.get("sensor_orientation") or z3)],
        "ping_range_sensor_location": [float(x) for x in (ping.get("sensor_location") or z3)],
        "ping_range_sensor_orientation": [float(x) for x in (ping.get("sensor_orientation") or z3)],
    }


def ekf_params_set(updates: Dict[str, Any]) -> Dict[str, Any]:
    path = vessel_yaml_path()
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    def fmt(v: Any) -> str:
        return f"{float(v):.6f}"

    def update_top(key: str, value: Any) -> bool:
        for i, line in enumerate(lines):
            m = NUM_RE.match(line.rstrip("\n"))
            if m and m.group(1) == key:
                indent = line[: len(line) - len(line.lstrip())]
                lines[i] = f"{indent}{key}: {fmt(value)}\n"
                return True
        return False

    def sensor_block(sensor_type: str) -> Tuple[Optional[int], Optional[int]]:
        start = None
        for i, line in enumerate(lines):
            m = SENSOR_RE.match(line.rstrip("\n"))
            if m and m.group(1) == sensor_type:
                start = i
                break
        if start is None:
            return None, None
        end = len(lines)
        for j in range(start + 1, len(lines)):
            if SENSOR_RE.match(lines[j].rstrip("\n")):
                end = j
                break
        return start, end

    def update_sensor(sensor_type: str, key: str, value: Any) -> bool:
        s, e = sensor_block(sensor_type)
        if s is None or e is None:
            return False
        for i in range(s + 1, e):
            m = NUM_RE.match(lines[i].rstrip("\n"))
            if m and m.group(1) == key:
                indent = lines[i][: len(lines[i]) - len(lines[i].lstrip())]
                lines[i] = f"{indent}{key}: {fmt(value)}\n"
                return True
        return False

    def update_sensor_array(sensor_type: str, key: str, arr: Sequence[Any]) -> bool:
        s, e = sensor_block(sensor_type)
        if s is None or e is None:
            return False
        arr_txt = ", ".join(fmt(v) for v in arr)
        for i in range(s + 1, e):
            ls = lines[i].rstrip("\n")
            if re.match(r"^\s*" + re.escape(key) + r":\s*\[.*\]\s*$", ls):
                indent = lines[i][: len(lines[i]) - len(lines[i].lstrip())]
                lines[i] = f"{indent}{key}: [{arr_txt}]\n"
                return True
        return False

    def update_imu(key: str, value: Any) -> bool:
        return update_sensor("IMU", key, value) or update_sensor("IMU_SBG", key, value)

    def update_imu_array(key: str, values: Any) -> bool:
        return update_sensor_array("IMU", key, values) or update_sensor_array("IMU_SBG", key, values)

    ok = True
    ok = update_top("time_step", updates["time_step"]) and ok
    ok = update_top("pool_depth", updates["pool_depth"]) and ok
    ok = update_top("dvl_depth_from_surface", updates["dvl_depth_from_surface"]) and ok
    ok = update_imu("default_gyro_variance", updates["imu_default_gyro_variance"]) and ok
    ok = update_imu("default_accel_variance", updates["imu_default_accel_variance"]) and ok
    ok = (
        update_sensor("ARDUINO_DEPTH", "default_variance", updates["arduino_depth_default_variance"])
        and ok
    )
    ok = update_sensor("DVL", "default_velocity_variance", updates["dvl_default_velocity_variance"]) and ok
    ok = (
        update_sensor("DVL_DR", "min_position_variance", updates["dvl_dr_min_position_variance"]) and ok
    )
    ok = (
        update_sensor("DVL_DR", "position_variance_scale", updates["dvl_dr_position_variance_scale"])
        and ok
    )
    ok = update_sensor("DVL_DR", "dr_drift_rate", updates["dvl_dr_drift_rate"]) and ok
    ok = update_sensor("PING_RANGE", "default_variance", updates["ping_range_default_variance"]) and ok
    ok = update_imu_array("sensor_location", updates["imu_sbg_sensor_location"]) and ok
    ok = update_imu_array("sensor_orientation", updates["imu_sbg_sensor_orientation"]) and ok
    ok = (
        update_sensor_array(
            "ARDUINO_DEPTH", "sensor_location", updates["arduino_depth_sensor_location"]
        )
        and ok
    )
    ok = (
        update_sensor_array(
            "ARDUINO_DEPTH", "sensor_orientation", updates["arduino_depth_sensor_orientation"]
        )
        and ok
    )
    ok = update_sensor_array("DVL", "sensor_location", updates["dvl_sensor_location"]) and ok
    ok = update_sensor_array("DVL", "sensor_orientation", updates["dvl_sensor_orientation"]) and ok
    ok = update_sensor_array("DVL_DR", "sensor_location", updates["dvl_dr_sensor_location"]) and ok
    ok = update_sensor_array("DVL_DR", "sensor_orientation", updates["dvl_dr_sensor_orientation"]) and ok
    ok = (
        update_sensor_array("PING_RANGE", "sensor_location", updates["ping_range_sensor_location"])
        and ok
    )
    ok = (
        update_sensor_array(
            "PING_RANGE", "sensor_orientation", updates["ping_range_sensor_orientation"]
        )
        and ok
    )
    if not ok:
        raise RuntimeError("Failed to update one or more EKF keys in vessel_data.example.yml")
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)
    return {"code": 0, "saved": True, "via": "vehicle-agent"}


def _rosbags_dir() -> str:
    return _path("rosbags")


def _safe_run_name(name: str) -> Optional[str]:
    cleaned = re.sub(r"[^a-zA-Z0-9_-]", "", str(name or ""))
    if cleaned and re.fullmatch(r"run\d+", cleaned):
        return cleaned
    return None


def _next_rosbag_run_name() -> str:
    root = _rosbags_dir()
    os.makedirs(root, exist_ok=True)
    max_n = 0
    try:
        for name in os.listdir(root):
            m = re.fullmatch(r"run(\d+)", name)
            if m:
                max_n = max(max_n, int(m.group(1)))
    except FileNotFoundError:
        pass
    return f"run{max_n + 1}"


def _record_pid_file(bag_name: str) -> str:
    # Keep outside the bag folder — writing into the output dir races rosbag2
    # (non-empty existing dir → recorder exits immediately).
    return os.path.join(_rosbags_dir(), f".mav_gui_record.{bag_name}.pids")


def _find_record_pids(bag_name: str) -> List[int]:
    """PIDs for ros2 bag record that write this run (not every recorder on the vehicle)."""
    markers = (f"/rosbags/{bag_name}", f"rosbags/{bag_name}")
    try:
        # Do not filter by -u here: vehicle_agent shares host PID ns; keep it simple.
        out = subprocess.check_output(
            ["bash", "-lc", "ps -eo pid=,args= 2>/dev/null || true"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return []
    pids: List[int] = []
    for ln in out.splitlines():
        if NOISE_RE.search(ln):
            continue
        low = ln.lower()
        # Real recorder looks like: python3 .../ros2 bag record ... -o .../rosbags/runN
        if "bag" not in low or "record" not in low:
            continue
        if "play" in low and "record" not in low.split():
            continue
        if not any(m in ln for m in markers):
            continue
        try:
            pids.append(int(ln.split(None, 1)[0]))
        except ValueError:
            continue
    return sorted(set(pids))


def _write_record_pids(bag_name: str, pids: List[int]) -> None:
    path = _record_pid_file(bag_name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(str(p) for p in pids))
        if pids:
            f.write("\n")


def _read_record_pids(bag_name: str) -> List[int]:
    path = _record_pid_file(bag_name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            return [int(x) for x in f.read().split() if x.isdigit()]
    except Exception:
        return []


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except Exception:
        return False


def _stop_record_run(bag_name: str) -> List[int]:
    """
    Force-stop the recorder for this run only.
    Kill by exact PID only — never pkill -f (that matches the kill shell itself and
    can suicide before the real ros2 bag record process receives a signal).
    """
    for attempt in range(10):
        pids = _find_record_pids(bag_name)
        if not pids:
            return []
        if attempt == 0:
            sig = signal.SIGINT
        elif attempt < 4:
            sig = signal.SIGTERM
        else:
            sig = signal.SIGKILL
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                # Fall back to kill(1) in case of capability quirks.
                subprocess.run(
                    ["bash", "-lc", f"kill -{int(sig)} {pid} >/dev/null 2>&1 || true"],
                    check=False,
                )
            except Exception:
                subprocess.run(
                    ["bash", "-lc", f"kill -{int(sig)} {pid} >/dev/null 2>&1 || true"],
                    check=False,
                )
        time.sleep(0.45 if attempt < 4 else 0.25)
    return _find_record_pids(bag_name)


def _recording_running_for(bag_name: Optional[str] = None) -> bool:
    if bag_name:
        return bool(_find_record_pids(bag_name))
    return False


def _discover_active_gui_recording() -> Tuple[Optional[str], List[int]]:
    """Find a GUI-started recorder via live cmdlines (not every bag record, never grep)."""
    root = _rosbags_dir()
    try:
        names = sorted(
            (n for n in os.listdir(root) if re.fullmatch(r"run\d+", n)),
            key=lambda n: int(n.replace("run", "")),
            reverse=True,
        )
    except FileNotFoundError:
        return None, []
    # Prefer newest runs. Only live `ros2 bag record …/rosbags/runN` counts —
    # ignore stale pid files and never treat `ps|grep` as a recorder.
    for name in names[:8]:
        pids = _find_record_pids(name)
        if pids:
            return name, pids
    return None, []


def rosbag_status() -> Dict[str, Any]:
    global _rosbag_active_name, _rosbag_active_pids
    name = _rosbag_active_name
    pids = list(_rosbag_active_pids)

    if name:
        live = _find_record_pids(name)
        if live:
            _rosbag_active_pids = live
            return {
                "recording": True,
                "bagName": name,
                "pids": live,
                "via": "vehicle-agent",
            }
        # Tracked run finished / was stopped.
        _rosbag_active_name = None
        _rosbag_active_pids = []
        return {
            "recording": False,
            "bagName": None,
            "pids": [],
            "via": "vehicle-agent",
        }

    # No in-memory track (agent restart) — only recover GUI recorders, never "any ros2 bag record".
    found_name, found_pids = _discover_active_gui_recording()
    if found_name:
        _rosbag_active_name = found_name
        _rosbag_active_pids = found_pids
        return {
            "recording": True,
            "bagName": found_name,
            "pids": found_pids,
            "via": "vehicle-agent",
        }

    return {
        "recording": False,
        "bagName": None,
        "pids": [],
        "via": "vehicle-agent",
    }


def rosbag_start(topics: List[str]) -> Dict[str, Any]:
    global _rosbag_active_name, _rosbag_active_pids
    if not topics:
        raise ValueError("topics required")
    # Refresh status first so a finished run doesn't block a new start.
    st = rosbag_status()
    if st.get("recording"):
        raise RuntimeError(
            f"rosbag recording already running ({st.get('bagName')}) — stop it first"
        )

    os.makedirs(_rosbags_dir(), exist_ok=True)
    bag_name = _next_rosbag_run_name()
    topic_list = " ".join(f"{t!r}" for t in topics)
    out = os.path.join(_rosbags_dir(), bag_name)
    start_detached(f"ros2 bag record {topic_list} -o {out!r}")

    pids: List[int] = []
    for _ in range(12):
        time.sleep(0.35)
        pids = _find_record_pids(bag_name)
        if pids:
            break

    _rosbag_active_name = bag_name
    _rosbag_active_pids = pids
    _write_record_pids(bag_name, pids)
    return {
        "code": 0,
        "stdout": f"recording started → rosbags/{bag_name}",
        "bagName": bag_name,
        "pids": pids,
        "via": "vehicle-agent",
    }


def rosbag_stop() -> Dict[str, Any]:
    global _rosbag_active_name, _rosbag_active_pids
    # Re-discover if memory was lost but a GUI recorder is still up.
    if not _rosbag_active_name:
        found_name, found_pids = _discover_active_gui_recording()
        if found_name:
            _rosbag_active_name = found_name
            _rosbag_active_pids = found_pids

    stopped = _rosbag_active_name
    pids_before = list(_rosbag_active_pids)
    if stopped:
        pids_before = pids_before or _read_record_pids(stopped) or _find_record_pids(stopped)

    still: List[int] = []
    if stopped:
        still = _stop_record_run(stopped)
        try:
            os.remove(_record_pid_file(stopped))
        except FileNotFoundError:
            pass
        except Exception:
            pass

    _rosbag_active_name = None
    _rosbag_active_pids = []
    return {
        "code": 0 if not still else 1,
        "stdout": (
            "recording stopped"
            if not still
            else f"recording stop incomplete — still running PIDs {still}"
        ),
        "bagName": stopped,
        "pids": pids_before,
        "recording": bool(still),
        "via": "vehicle-agent",
    }


def rosbag_runs() -> Dict[str, Any]:
    root = _rosbags_dir()
    runs: List[str] = []
    try:
        for name in os.listdir(root):
            if re.fullmatch(r"run\d+", name):
                runs.append(name)
    except FileNotFoundError:
        pass
    runs.sort(key=lambda n: int(n.replace("run", "")), reverse=True)
    notes: Dict[str, str] = {}
    for run_name in runs:
        note_path = os.path.join(root, run_name, "mav_gui_note.txt")
        try:
            with open(note_path, "r", encoding="utf-8", errors="replace") as f:
                notes[run_name] = f.read().replace("\r\n", "\n").rstrip("\n")
        except FileNotFoundError:
            notes[run_name] = ""
        except Exception:
            notes[run_name] = ""
    return {"runs": runs, "notes": notes, "via": "vehicle-agent"}


def rosbag_label(run_name: str, note: str) -> Dict[str, Any]:
    rn = _safe_run_name(run_name)
    if not rn:
        raise ValueError("invalid run name")
    out_path = os.path.join(_rosbags_dir(), rn, "mav_gui_note.txt")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(note)
    return {"code": 0, "note": note, "via": "vehicle-agent"}


def rosbag_metadata(run_name: str) -> Dict[str, Any]:
    rn = _safe_run_name(run_name)
    if not rn:
        raise ValueError("invalid run name")
    meta_path = os.path.join(_rosbags_dir(), rn, "metadata.yaml")
    with open(meta_path, "r", encoding="utf-8", errors="replace") as f:
        return {"metadata": f.read(), "via": "vehicle-agent"}


def rosbag_delete(run_name: str) -> Dict[str, Any]:
    rn = _safe_run_name(run_name)
    if not rn:
        raise ValueError("invalid run name")
    target = os.path.join(_rosbags_dir(), rn)
    if not os.path.isdir(target):
        raise FileNotFoundError(f"missing {rn}")
    shutil.rmtree(target)
    return {"code": 0, "stdout": f"deleted {rn}", "via": "vehicle-agent"}


def rosbag_play(run_name: str) -> Dict[str, Any]:
    global _bag_active_run
    rn = _safe_run_name(run_name)
    if not rn:
        raise ValueError("invalid run name")
    bag_path = os.path.join(_rosbags_dir(), rn)
    if not os.path.isdir(bag_path):
        raise FileNotFoundError(f"missing {rn}")
    subprocess.run(
        ["bash", "-lc", 'pkill -INT -f "ros2 bag play" >/dev/null 2>&1 || true'],
        check=False,
    )
    start_detached(f"ros2 bag play {bag_path!r} --clock")
    _bag_active_run = rn
    return {"code": 0, "run": rn, "via": "vehicle-agent"}


def rosbag_playback_stop() -> Dict[str, Any]:
    global _bag_active_run
    subprocess.run(
        ["bash", "-lc", 'pkill -INT -f "ros2 bag play" >/dev/null 2>&1 || true'],
        check=False,
    )
    stop_pattern("ros2 bag play")
    stopped = _bag_active_run
    _bag_active_run = None
    return {"code": 0, "stopped": True, "run": stopped, "via": "vehicle-agent"}


def read_json_body(handler: Any) -> Dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    raw = handler.rfile.read(length) if length > 0 else b"{}"
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))


def handle_get(path: str) -> Optional[Tuple[int, Dict[str, Any]]]:
    if path == "/nav/status":
        return 200, nav_status()
    if path == "/nav/ekf-params":
        return 200, ekf_params_get()
    if path == "/control2d/status":
        return 200, control_status(False)
    if path == "/control2d/pid":
        return 200, control_pid_get(False)
    if path == "/control3d/status":
        return 200, control_status(True)
    if path == "/control3d/pid":
        return 200, control_pid_get(True)
    if path == "/rosbag/status":
        return 200, rosbag_status()
    if path == "/rosbag/runs":
        return 200, rosbag_runs()
    m_meta = re.match(r"^/rosbag/runs/([^/]+)/metadata$", path)
    if m_meta:
        try:
            return 200, rosbag_metadata(m_meta.group(1))
        except FileNotFoundError as e:
            return 404, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
        except ValueError as e:
            return 400, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}

    m = re.match(
        r"^/(missions|missions3d)/(point-tracking|point-tracking-los|depth-control|station-keeping|dwell)/(params|status)$",
        path,
    )
    if m:
        prefix, mode, action = m.group(1), m.group(2), m.group(3)
        kind = _mission_kind(prefix, mode)
        if action == "status":
            return 200, mission_status(kind)
        return 200, mission_params_get(kind)
    return None


def handle_post(path: str, body: Dict[str, Any]) -> Optional[Tuple[int, Dict[str, Any]]]:
    try:
        if path == "/nav/start":
            mode = str(body.get("mode") or "live")
            return 200, nav_start(mode, body.get("bagName"))
        if path == "/nav/stop":
            return 200, nav_stop()
        if path == "/nav/ekf-params":
            return 200, ekf_params_set(body)
        if path == "/control2d/start":
            return 200, control_start(False)
        if path == "/control2d/stop":
            return 200, control_stop(False)
        if path == "/control2d/pid":
            return 200, control_pid_set(False, body)
        if path == "/control3d/start":
            return 200, control_start(True)
        if path == "/control3d/stop":
            return 200, control_stop(True)
        if path == "/control3d/pid":
            return 200, control_pid_set(True, body)
        if path == "/rosbag/start":
            topics = body.get("topics") or []
            if not isinstance(topics, list) or not topics:
                return 400, {"code": 1, "stderr": "topics required", "via": "vehicle-agent"}
            try:
                return 200, rosbag_start([str(t) for t in topics])
            except RuntimeError as e:
                return 409, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
        if path == "/rosbag/stop":
            return 200, rosbag_stop()
        if path == "/rosbag/playback/stop":
            return 200, rosbag_playback_stop()
        m_play = re.match(r"^/rosbag/runs/([^/]+)/play$", path)
        if m_play:
            return 200, rosbag_play(m_play.group(1))

        m = re.match(
            r"^/(missions|missions3d)/(point-tracking|point-tracking-los|depth-control|station-keeping|dwell)/(params|start|stop)$",
            path,
        )
        if m:
            prefix, mode, action = m.group(1), m.group(2), m.group(3)
            kind = _mission_kind(prefix, mode)
            if action == "stop":
                return 200, mission_stop(kind)
            if action == "params":
                return 200, mission_params_set(kind, body)
            return 200, mission_start(kind, body if isinstance(body, dict) else {})
    except ValueError as e:
        return 400, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    except FileNotFoundError as e:
        return 404, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    except Exception as e:
        return 500, {"code": 1, "stderr": str(e), "error": str(e), "via": "vehicle-agent"}
    return None


def handle_put(path: str, body: Dict[str, Any]) -> Optional[Tuple[int, Dict[str, Any]]]:
    try:
        m = re.match(r"^/rosbag/runs/([^/]+)/label$", path)
        if m:
            note = str(body.get("note") or "")
            if len(note) > 4000:
                return 400, {"code": 1, "stderr": "note too long", "via": "vehicle-agent"}
            return 200, rosbag_label(m.group(1), note)
    except ValueError as e:
        return 400, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    except Exception as e:
        return 500, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    return None


def handle_delete(path: str) -> Optional[Tuple[int, Dict[str, Any]]]:
    try:
        m = re.match(r"^/rosbag/runs/([^/]+)$", path)
        if m:
            return 200, rosbag_delete(m.group(1))
    except ValueError as e:
        return 400, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    except FileNotFoundError as e:
        return 404, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    except Exception as e:
        return 500, {"code": 1, "stderr": str(e), "via": "vehicle-agent"}
    return None


def _mission_kind(prefix: str, mode: str) -> str:
    if mode == "depth-control":
        if prefix != "missions3d":
            raise ValueError("depth-control is 3D only")
        return "3d_depth"
    if mode == "station-keeping":
        if prefix != "missions3d":
            raise ValueError("station-keeping is 3D only")
        return "3d_station"
    if mode == "dwell":
        if prefix != "missions3d":
            raise ValueError("dwell is 3D only")
        return "3d_dwell"
    dim = "3d" if prefix == "missions3d" else "2d"
    guidance = "los" if mode.endswith("-los") else "ilos"
    return f"{dim}_{guidance}"
