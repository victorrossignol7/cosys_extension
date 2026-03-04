# orchestrator_server.py
import json
import socketserver
import threading
import time
from typing import Any, Dict, Optional

import cosysairsim as airsim

HOST = "127.0.0.1"
PORT = 50051

# Namespace routing: topic -> vehicle name
TOPIC_TO_VEHICLE = {
    "/uav1/cmd": "Drone1",
    "/uav2/cmd": "Drone2",
    "/uav1/odom": "Drone1",
    "/uav2/odom": "Drone2",
}

_clients: Dict[str, airsim.MultirotorClient] = {}
_locks: Dict[str, threading.Lock] = {}


def get_lock(vehicle: str) -> threading.Lock:
    if vehicle not in _locks:
        _locks[vehicle] = threading.Lock()
    return _locks[vehicle]


def get_client(vehicle: str) -> airsim.MultirotorClient:
    if vehicle in _clients:
        return _clients[vehicle]
    c = airsim.MultirotorClient()
    c.confirmConnection()
    c.enableApiControl(True, vehicle)
    c.armDisarm(True, vehicle)
    _clients[vehicle] = c
    return c


def route_vehicle(topic: Optional[str], vehicle: Optional[str]) -> str:
    if vehicle:
        return vehicle
    if topic and topic in TOPIC_TO_VEHICLE:
        return TOPIC_TO_VEHICLE[topic]
    raise ValueError("Missing vehicle and unknown topic")


def state_snapshot(client: airsim.MultirotorClient, vehicle: str) -> Dict[str, Any]:
    s = client.getMultirotorState(vehicle_name=vehicle)
    p = s.kinematics_estimated.position
    v = s.kinematics_estimated.linear_velocity
    return {
        "pos": {"x": p.x_val, "y": p.y_val, "z": p.z_val, "alt": -p.z_val},  # NED -> altitude
        "vel": {"x": v.x_val, "y": v.y_val, "z": v.z_val},
        "ts_wall": time.time(),
    }


def imu_snapshot(client: airsim.MultirotorClient, vehicle: str) -> Dict[str, Any]:
    imu = client.getImuData(vehicle_name=vehicle)
    w = imu.angular_velocity
    a = imu.linear_acceleration
    return {
        "ang_vel": {"x": w.x_val, "y": w.y_val, "z": w.z_val},
        "lin_acc": {"x": a.x_val, "y": a.y_val, "z": a.z_val},
        "ts_wall": time.time(),
    }


def handle_op(vehicle: str, op: str, args: Dict[str, Any]) -> Dict[str, Any]:
    lock = get_lock(vehicle)
    with lock:
        client = get_client(vehicle)

        if op == "takeoff":
            client.takeoffAsync(vehicle_name=vehicle).join()
        elif op == "land":
            client.landAsync(vehicle_name=vehicle).join()
        elif op == "moveToZ":
            z = float(args.get("z", -2.0))
            speed = float(args.get("speed", 1.5))
            client.moveToZAsync(z, speed, vehicle_name=vehicle).join()
        elif op == "moveByVelocityZ":
            vx = float(args.get("vx", 0.0))
            vy = float(args.get("vy", 0.0))
            z = float(args.get("z", -2.0))
            duration = float(args.get("duration", 0.5))
            yaw_rate = float(args.get("yaw_rate", 0.0))
            client.moveByVelocityZAsync(
                vx=vx,
                vy=vy,
                z=z,
                duration=duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                vehicle_name=vehicle,
            ).join()
        elif op in ("get_state", "get_imu"):
            pass
        else:
            raise ValueError(f"Unknown op: {op}")

        resp = {"vehicle": vehicle, "op": op, "state": state_snapshot(client, vehicle)}
        if op in ("get_imu", "takeoff", "moveToZ", "moveByVelocityZ"):
            resp["imu"] = imu_snapshot(client, vehicle)
        return resp


class Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        peer = self.client_address
        print(f"[server] client connected: {peer}")

        while True:
            line = self.rfile.readline()
            if not line:
                break

            try:
                msg = json.loads(line.decode("utf-8"))
                topic = msg.get("topic")
                vehicle = route_vehicle(topic, msg.get("vehicle"))
                op = msg.get("op", "get_state")
                args = msg.get("args", {}) or {}

                out = handle_op(vehicle, op, args)
                out["ok"] = True
            except Exception as e:
                out = {"ok": False, "error": str(e)}

            self.wfile.write((json.dumps(out) + "\n").encode("utf-8"))

        print(f"[server] client disconnected: {peer}")


def main() -> None:
    server = socketserver.ThreadingTCPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    print(f"[server] orchestrator listening on {HOST}:{PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
