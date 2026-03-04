#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COSYS/AirSim dual-robot demo terminal (Windows-friendly, colored) + CSV logging.

Features
- Launches test_drone.py in background
- Shows UAV + UGV connectivity and live positions
- Displays configured sensors/cameras status from settings.json (OK/OFF)
- Logs sensor telemetry to CSV (one CSV per robot) with timestamp and units in headers
- Compact colored terminal UI without curses (works in PowerShell)

Usage:
  python demo_cosys_dual_terminal_color_csv.py [test_drone.py] [settings.json]

Press:
  q + Enter (Unix) OR q (Windows) to quit
"""
from __future__ import annotations

import csv
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ANSI colors (PowerShell supports ANSI on modern Windows Terminal)
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
GRAY = "\033[90m"

# ------------------------ Optional AirSim import ------------------------
try:
    import cosysairsim as airsim
except Exception as e:
    print(f"[FATAL] Could not import cosysairsim: {e}")
    sys.exit(1)

# ------------------------ Defaults from user's files ------------------------
DEFAULT_HOST = "127.0.0.1"
UAV_PORT = 41451
UGV_PORT = 41452
UAV_NAME = "Drone1"
UGV_NAME = "UGV1"

REFRESH_HZ = 5.0          # screen refresh
LOG_HZ = 5.0              # CSV logging rate
CONNECTION_RETRY_S = 1.0
MAX_LOG_LINES = 10

SENSOR_TYPE_MAP = {
    1: "Barometer",
    2: "Imu",
    3: "Gps",
    4: "Magnetometer",
    5: "Distance",
    6: "Lidar",
}

COMMON_UNITS = {
    # position / pose
    "position.x_val": "m", "position.y_val": "m", "position.z_val": "m",
    "linear_velocity.x_val": "m/s", "linear_velocity.y_val": "m/s", "linear_velocity.z_val": "m/s",
    "angular_velocity.x_val": "rad/s", "angular_velocity.y_val": "rad/s", "angular_velocity.z_val": "rad/s",
    "linear_acceleration.x_val": "m/s^2", "linear_acceleration.y_val": "m/s^2", "linear_acceleration.z_val": "m/s^2",
    "orientation.w_val": "", "orientation.x_val": "", "orientation.y_val": "", "orientation.z_val": "",
    # AirSim GPS names may vary
    "gnss.geo_point.latitude": "deg",
    "gnss.geo_point.longitude": "deg",
    "gnss.geo_point.altitude": "m",
    "geo_point.latitude": "deg",
    "geo_point.longitude": "deg",
    "geo_point.altitude": "m",
    "latitude": "deg",
    "longitude": "deg",
    "altitude": "m",
    # common scalar names
    "altitude": "m",
    "pressure": "Pa",
    "qnh": "Pa",
    "temperature": "C",
    "time_utc": "s",
    "speed": "m/s",
    "rpm": "rpm",
    # lidar summaries
    "point_count": "count",
    "range_min": "m",
    "range_max": "m",
}


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def try_call(obj, name, *args, **kwargs):
    if not hasattr(obj, name):
        return None, f"missing:{name}"
    fn = getattr(obj, name)
    try:
        return fn(*args, **kwargs), None
    except TypeError:
        try:
            return fn(*args), None
        except Exception as e:
            return None, str(e)
    except Exception as e:
        return None, str(e)


def fnum(v: Any, nd: int = 2) -> str:
    try:
        return f"{float(v):.{nd}f}"
    except Exception:
        return "?"


def clear_screen():
    # Cross-platform clear + home
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def extract_scalar_fields(obj: Any, prefix: str = "", depth: int = 0, max_depth: int = 4) -> Dict[str, float]:
    """Recursively flatten AirSim-like objects into scalar numeric fields.
    Skips huge arrays (e.g., lidar point clouds) except we summarize separately.
    """
    out: Dict[str, float] = {}
    if obj is None or depth > max_depth:
        return out

    # Scalars
    if isinstance(obj, bool):
        out[prefix[:-1] if prefix.endswith('.') else prefix or 'value'] = 1.0 if obj else 0.0
        return out
    if isinstance(obj, (int, float)) and not isinstance(obj, bool):
        out[prefix[:-1] if prefix.endswith('.') else prefix or 'value'] = float(obj)
        return out

    # Strings / bytes not logged as scalar telemetry
    if isinstance(obj, (str, bytes)):
        return out

    # Lists/tuples: don't expand large arrays (lidar point_cloud). Keep summary elsewhere.
    if isinstance(obj, (list, tuple)):
        if len(obj) <= 8 and all(isinstance(x, (int, float, bool)) for x in obj):
            for i, v in enumerate(obj):
                out[f"{prefix}{i}"] = float(v)
        return out

    # Dicts
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k in ("point_cloud", "image_data_uint8", "image_data_float"):
                continue
            out.update(extract_scalar_fields(v, prefix=f"{prefix}{k}.", depth=depth+1, max_depth=max_depth))
        return out

    # Generic object attributes
    for attr in dir(obj):
        if attr.startswith("_"):
            continue
        if attr in ("to_msgpack", "from_msgpack"):
            continue
        try:
            v = getattr(obj, attr)
        except Exception:
            continue
        if callable(v):
            continue
        if attr in ("point_cloud", "image_data_uint8", "image_data_float"):
            continue
        out.update(extract_scalar_fields(v, prefix=f"{prefix}{attr}.", depth=depth+1, max_depth=max_depth))
    return out


def summarize_lidar_data(lidar_obj: Any) -> Dict[str, float]:
    d: Dict[str, float] = {}
    if lidar_obj is None:
        return d
    # point cloud can be flat list xyzxyz...
    pc = None
    try:
        pc = getattr(lidar_obj, "point_cloud", None)
    except Exception:
        pc = None
    if pc is not None:
        try:
            n = len(pc) // 3
            d["point_count"] = float(n)
        except Exception:
            pass
    # include pose-ish fields if present (small)
    d.update({k: v for k, v in extract_scalar_fields(lidar_obj).items() if any(s in k for s in ["pose.position", "pose.orientation", "time_stamp"])})
    return d


def infer_unit(sensor_name: str, field_name: str) -> str:
    # direct exact mapping first
    if field_name in COMMON_UNITS:
        return COMMON_UNITS[field_name]
    # suffix heuristics
    low = field_name.lower()
    if low.endswith("latitude") or low.endswith("longitude"):
        return "deg"
    if low.endswith("altitude") or low.endswith("_alt"):
        return "m"
    if "acceleration" in low:
        return "m/s^2"
    if "angular_velocity" in low or low.endswith("gyro"):
        return "rad/s"
    if "magnetic_field" in low or "magnet" in sensor_name.lower():
        return "T"
    if "pressure" in low:
        return "Pa"
    if "temperature" in low:
        return "C"
    if low.endswith("speed"):
        return "m/s"
    return ""


@dataclass
class RobotConfig:
    name: str
    kind: str  # UAV / UGV
    port: int
    vehicle_type: str = "?"
    sensors: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    cameras: Dict[str, Dict[str, Any]] = field(default_factory=dict)


class DynamicCsvLogger:
    """One wide CSV per robot, dynamic columns. Units are included in header labels.
    If schema changes, file is rotated and a new CSV with incremented suffix is created.
    """
    def __init__(self, out_dir: Path, robot_name: str):
        self.out_dir = out_dir
        self.robot_name = robot_name
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.schema: List[str] = []
        self.file_index = 1
        self.fp = None
        self.writer = None

    def _open_new_file(self, columns: List[str]):
        if self.fp:
            self.fp.close()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.out_dir / f"{self.robot_name}_telemetry_{ts}_part{self.file_index}.csv"
        self.file_index += 1
        self.fp = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.fp, fieldnames=columns)
        self.writer.writeheader()
        self.fp.flush()
        self.schema = columns
        self.current_path = path

    def log(self, row_base: Dict[str, Any], sensor_values: Dict[str, Tuple[float, str]]):
        # Build labels with units in header
        flat = dict(row_base)
        for key, (val, unit) in sensor_values.items():
            label = f"{key} [{unit}]" if unit else key
            flat[label] = val
        columns = list(flat.keys())
        if self.writer is None:
            self._open_new_file(columns)
        elif columns != self.schema:
            # rotate on schema change to avoid rewriting old CSV
            self._open_new_file(columns)
        self.writer.writerow(flat)
        self.fp.flush()

    def close(self):
        try:
            if self.fp:
                self.fp.close()
        except Exception:
            pass


class StreamReaderThread(threading.Thread):
    def __init__(self, proc: subprocess.Popen, q: queue.Queue, tag: str):
        super().__init__(daemon=True)
        self.proc = proc
        self.q = q
        self.tag = tag

    def run(self):
        try:
            if self.proc.stdout is None:
                return
            for line in self.proc.stdout:
                self.q.put((self.tag, line.rstrip("\n")))
        except Exception as e:
            self.q.put((self.tag, f"[reader-error] {e}"))


class RobotRuntime:
    def __init__(self, cfg: RobotConfig, log_dir: Path):
        self.cfg = cfg
        self.client = None
        self.connected = False
        self.last_error: Optional[str] = None
        self.last_pose = None
        self.last_pose_ts = 0.0
        self.last_state: Dict[str, Any] = {}
        self.last_sensor_sample: Dict[str, Tuple[float, str]] = {}
        self.csv = DynamicCsvLogger(log_dir, cfg.name)

    def connect(self):
        try:
            if self.cfg.kind == "UAV":
                c = airsim.MultirotorClient(ip=DEFAULT_HOST, port=self.cfg.port)
            else:
                c = airsim.CarClient(ip=DEFAULT_HOST, port=self.cfg.port)
            c.confirmConnection()
            self.client = c
            self.connected = True
            self.last_error = None
        except Exception as e:
            self.client = None
            self.connected = False
            self.last_error = str(e)

    def ensure_connected(self):
        if not self.connected or self.client is None:
            self.connect()

    def _get_pose(self):
        if not self.client:
            return None
        for fn in ("simGetVehiclePose", "getVehiclePose"):
            p, err = try_call(self.client, fn, vehicle_name=self.cfg.name)
            if err is None and p is not None:
                return p
        return None

    def _get_basic_state(self) -> Dict[str, Any]:
        st = {}
        if not self.client:
            return st
        if self.cfg.kind == "UAV":
            ms, err = try_call(self.client, "getMultirotorState", vehicle_name=self.cfg.name)
            if err is None and ms is not None:
                # speed norm if available
                kin = getattr(ms, "kinematics_estimated", None)
                if kin is not None:
                    lv = getattr(kin, "linear_velocity", None)
                    if lv is not None:
                        try:
                            st["speed"] = math.sqrt(lv.x_val**2 + lv.y_val**2 + lv.z_val**2)
                        except Exception:
                            pass
                landed = getattr(ms, "landed_state", None)
                if landed is not None:
                    st["landed_state"] = landed
        else:
            cs, err = try_call(self.client, "getCarState", vehicle_name=self.cfg.name)
            if err is None and cs is not None:
                for k in ("speed", "gear", "rpm"):
                    if hasattr(cs, k):
                        try:
                            st[k] = getattr(cs, k)
                        except Exception:
                            pass
        return st

    def _sample_sensor(self, sensor_name: str, sensor_cfg: Dict[str, Any]) -> Dict[str, Tuple[float, str]]:
        """Returns {'Sensor.field': (value, unit)}."""
        out: Dict[str, Tuple[float, str]] = {}
        if not self.client:
            return out
        enabled = bool(sensor_cfg.get("Enabled", True))
        if not enabled:
            return out

        sensor_type = sensor_cfg.get("SensorType")
        logical_type = SENSOR_TYPE_MAP.get(sensor_type, sensor_name)

        api_candidates = []
        t = logical_type.lower()
        if "imu" in t:
            api_candidates = ["getImuData"]
        elif "gps" in t:
            api_candidates = ["getGpsData"]
        elif "baro" in t:
            api_candidates = ["getBarometerData"]
        elif "mag" in t:
            api_candidates = ["getMagnetometerData"]
        elif "lidar" in t:
            api_candidates = ["getLidarData"]
        else:
            api_candidates = [f"get{logical_type}Data"]

        data = None
        err = "no_api"
        for api in api_candidates:
            # Common signature: (sensor_name=..., vehicle_name=...)
            data, err = try_call(self.client, api, sensor_name=sensor_name, vehicle_name=self.cfg.name)
            if err is None and data is not None:
                break
            # Compatibility attempt without keywords
            data, err = try_call(self.client, api, sensor_name, self.cfg.name)
            if err is None and data is not None:
                break
        if data is None:
            return out

        if "lidar" in t:
            flat = summarize_lidar_data(data)
        else:
            flat = extract_scalar_fields(data)

        # filter to numeric, useful fields only
        for field, value in flat.items():
            if not isinstance(value, (int, float)):
                continue
            key = f"{sensor_name}.{field}"
            unit = infer_unit(sensor_name, field)
            out[key] = (float(value), unit)
        return out

    def poll(self):
        self.ensure_connected()
        if not self.connected:
            return
        try:
            self.last_pose = self._get_pose()
            self.last_pose_ts = time.time()
            self.last_state = self._get_basic_state()
            sample: Dict[str, Tuple[float, str]] = {}
            for sname, scfg in self.cfg.sensors.items():
                sample.update(self._sample_sensor(sname, scfg))
            self.last_sensor_sample = sample
            # Log one wide row per robot
            row_base = {
                "timestamp_iso": now_iso(),
                "timestamp_epoch_s": round(time.time(), 3),
            }
            # Always include pose and basic state if available
            if self.last_pose is not None:
                try:
                    row_base["pose_x [m]"] = self.last_pose.position.x_val
                    row_base["pose_y [m]"] = self.last_pose.position.y_val
                    row_base["pose_z [m]"] = self.last_pose.position.z_val
                except Exception:
                    pass
            for k, v in self.last_state.items():
                unit = infer_unit("state", k)
                row_base[f"state.{k}{(' ['+unit+']') if unit else ''}"] = v
            self.csv.log(row_base, sample)
            self.connected = True
            self.last_error = None
        except Exception as e:
            self.connected = False
            self.last_error = str(e)

    def close(self):
        self.csv.close()


def load_settings(settings_path: Path) -> Dict[str, RobotConfig]:
    raw = json.loads(settings_path.read_text(encoding="utf-8"))
    vehicles = raw.get("Vehicles", {})
    out: Dict[str, RobotConfig] = {}
    for name, v in vehicles.items():
        vtype = str(v.get("VehicleType", "?"))
        kind = "UAV" if any(k in vtype.lower() for k in ["flight", "drone", "multirotor"]) else "UGV"
        port = UAV_PORT if kind == "UAV" else UGV_PORT
        out[name] = RobotConfig(
            name=name,
            kind=kind,
            port=port,
            vehicle_type=vtype,
            sensors=v.get("Sensors", {}) or {},
            cameras=v.get("Cameras", {}) or {},
        )
    return out


def render_status_line(label: str, ok: bool, extra: str = "") -> str:
    color = GREEN if ok else RED
    text = "OK" if ok else "ERR"
    return f"{BOLD}{label:<12}{RESET} {color}{text:<3}{RESET} {extra}"


def robot_panel(rt: RobotRuntime) -> List[str]:
    cfg = rt.cfg
    color = CYAN if cfg.kind == "UAV" else MAGENTA
    lines = []
    head_status = f"{GREEN}CONNECTED{RESET}" if rt.connected else f"{RED}DISCONNECTED{RESET}"
    lines.append(f"{BOLD}{color}{cfg.kind} {cfg.name}{RESET}  ({cfg.vehicle_type})  {head_status}  {GRAY}@{DEFAULT_HOST}:{cfg.port}{RESET}")
    if rt.last_error:
        lines.append(f"  {RED}last_error:{RESET} {rt.last_error}")

    # Pose
    if rt.last_pose is not None:
        try:
            p = rt.last_pose.position
            lines.append(f"  {BOLD}Pose{RESET}  x={fnum(p.x_val)} m   y={fnum(p.y_val)} m   z={fnum(p.z_val)} m")
        except Exception:
            lines.append("  Pose  ?")
    else:
        lines.append(f"  {BOLD}Pose{RESET}  ?")

    # Basic state compact
    if rt.last_state:
        kv = []
        for k in ["speed", "gear", "rpm", "landed_state"]:
            if k in rt.last_state:
                val = rt.last_state[k]
                if isinstance(val, float):
                    kv.append(f"{k}={val:.2f}")
                else:
                    kv.append(f"{k}={val}")
        if kv:
            lines.append("  " + f"{BOLD}State{RESET} " + "   ".join(kv))

    # Sensors and cameras statuses
    sensor_chunks = []
    for sname, scfg in cfg.sensors.items():
        enabled = bool(scfg.get("Enabled", True))
        c = GREEN if enabled else GRAY
        symbol = "●" if enabled else "○"
        st = "OK" if enabled else "OFF"
        sensor_chunks.append(f"{c}{symbol} {sname}({st}){RESET}")
    cam_chunks = []
    for cname in cfg.cameras.keys():
        cam_chunks.append(f"{BLUE}◉ {cname}(OK){RESET}")
    if sensor_chunks:
        lines.append(f"  {BOLD}Sensors{RESET} " + "  ".join(sensor_chunks))
    if cam_chunks:
        lines.append(f"  {BOLD}Cameras{RESET} " + "  ".join(cam_chunks))

    # Show few live sensor values (not too crowded)
    preview_items = []
    preferred = [
        "Imu.linear_acceleration.x_val", "Imu.linear_acceleration.y_val", "Imu.linear_acceleration.z_val",
        "Gps.gnss.geo_point.latitude", "Gps.gnss.geo_point.longitude",
        "Barometer.pressure", "Magnetometer.magnetic_field_body.x_val", "Lidar1.point_count",
    ]
    bykey = rt.last_sensor_sample
    for prefix in preferred:
        for key, (val, unit) in bykey.items():
            if prefix in key:
                preview_items.append(f"{key.split('.',1)[0]}:{key.split('.')[-1]}={val:.2f}{(' '+unit) if unit else ''}")
                break
    if not preview_items:
        # fallback first 4 items
        for i, (key, (val, unit)) in enumerate(bykey.items()):
            if i >= 4:
                break
            preview_items.append(f"{key}={val:.2f}{(' '+unit) if unit else ''}")
    if preview_items:
        lines.append(f"  {BOLD}Live sensors{RESET} " + " | ".join(preview_items[:4]))

    # CSV path
    path = getattr(rt.csv, "current_path", None)
    if path:
        lines.append(f"  {BOLD}CSV{RESET}  {GRAY}{path}{RESET}")
    return lines


def read_nonblocking_quit() -> bool:
    if os.name == "nt":
        try:
            import msvcrt
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch.lower() == "q":
                    return True
        except Exception:
            return False
        return False
    else:
        # simple stdin line mode fallback: q + Enter
        return False


def main():
    script_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("test_drone.py")
    settings_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("settings.json")

    if not script_path.exists():
        print(f"[FATAL] Drone script not found: {script_path}")
        sys.exit(1)
    if not settings_path.exists():
        print(f"[FATAL] settings.json not found: {settings_path}")
        sys.exit(1)

    cfgs = load_settings(settings_path)
    # Focus on Drone1 / UGV1 if present, otherwise first UAV/UGV found
    chosen: List[RobotConfig] = []
    if UAV_NAME in cfgs:
        chosen.append(cfgs[UAV_NAME])
    else:
        uavs = [c for c in cfgs.values() if c.kind == "UAV"]
        if uavs:
            chosen.append(uavs[0])
    if UGV_NAME in cfgs:
        chosen.append(cfgs[UGV_NAME])
    else:
        ugvs = [c for c in cfgs.values() if c.kind == "UGV"]
        if ugvs:
            chosen.append(ugvs[0])

    log_dir = Path("demo_logs")
    runtimes = [RobotRuntime(c, log_dir) for c in chosen]

    # Launch drone script
    proc = subprocess.Popen(
        [sys.executable, str(script_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    q_logs: queue.Queue = queue.Queue()
    StreamReaderThread(proc, q_logs, "drone-script").start()
    log_lines: deque[str] = deque(maxlen=MAX_LOG_LINES)
    log_lines.append(f"[{now_iso()}] launched {script_path.name} (pid={proc.pid})")

    # Unix stdin watcher (q + Enter)
    stop_flag = {"stop": False}
    def stdin_waiter():
        if os.name == "nt":
            return
        try:
            for line in sys.stdin:
                if line.strip().lower() == "q":
                    stop_flag["stop"] = True
                    break
        except Exception:
            pass

    threading.Thread(target=stdin_waiter, daemon=True).start()

    last_poll = 0.0
    poll_period = 1.0 / LOG_HZ
    refresh_period = 1.0 / REFRESH_HZ

    try:
        while True:
            t0 = time.time()

            # Quit handling
            if stop_flag["stop"] or read_nonblocking_quit():
                break

            # Drain logs queue
            while True:
                try:
                    tag, line = q_logs.get_nowait()
                except queue.Empty:
                    break
                log_lines.append(f"[{tag}] {line}")

            # Poll robots
            if t0 - last_poll >= poll_period:
                for rt in runtimes:
                    rt.poll()
                last_poll = t0

            # Render
            clear_screen()
            title = f"{BOLD}{WHITE}COSYS / AirSim Dual-Robot Demo Monitor{RESET}"
            subtitle = f"{GRAY}{now_iso()}  |  q to quit  |  logs -> {Path('demo_logs').resolve()}{RESET}"
            print(title)
            print(subtitle)
            print(f"{GRAY}{'='*100}{RESET}")

            # Drone script status
            proc_alive = proc.poll() is None
            pcol = GREEN if proc_alive else YELLOW
            ptxt = "RUNNING" if proc_alive else f"EXITED ({proc.returncode})"
            print(f"{BOLD}Drone script{RESET}  {pcol}{ptxt}{RESET}  {GRAY}{script_path}{RESET}")
            print()

            # Panels
            for idx, rt in enumerate(runtimes):
                for line in robot_panel(rt):
                    print(line)
                if idx != len(runtimes)-1:
                    print(f"{GRAY}{'-'*100}{RESET}")
            print()
            print(f"{BOLD}Recent logs ({MAX_LOG_LINES}){RESET}")
            for line in list(log_lines)[-MAX_LOG_LINES:]:
                if "[ok]" in line.lower() or "[done]" in line.lower():
                    c = GREEN
                elif "error" in line.lower() or "traceback" in line.lower():
                    c = RED
                elif "[uav]" in line.lower():
                    c = CYAN
                else:
                    c = GRAY
                print(f"{c}{line}{RESET}")

            # maintain refresh rate
            dt = time.time() - t0
            time.sleep(max(0.02, refresh_period - dt))

    except KeyboardInterrupt:
        pass
    finally:
        for rt in runtimes:
            rt.close()
        try:
            if proc.poll() is None:
                if os.name == "nt":
                    proc.send_signal(signal.CTRL_BREAK_EVENT) if hasattr(signal, 'CTRL_BREAK_EVENT') else proc.terminate()
                    time.sleep(0.5)
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except Exception:
                    proc.kill()
        except Exception:
            pass
        print("\nExiting demo monitor.")


if __name__ == "__main__":
    main()
