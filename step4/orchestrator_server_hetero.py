#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import queue
import asyncio
import threading
import socketserver

import cosysairsim as airsim

IP = "127.0.0.1"
UAV_PORT = 41451
UGV_PORT = 41452
UAV_NAME = "Drone1"
UGV_NAME = "UGV1"
HOST, PORT = "127.0.0.1", 50051

# Jobs executed by the single AirSim worker
jobs: "queue.Queue[tuple]" = queue.Queue()

def make_reply(ok=True, **kw) -> bytes:
    d = {"ok": ok}
    d.update(kw)
    return (json.dumps(d, ensure_ascii=False) + "\n").encode("utf-8")

def airsim_worker():
    # Ensure this thread has an asyncio loop object (some libs call get_event_loop)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    print(f"[worker] connecting UAV {IP}:{UAV_PORT} name={UAV_NAME}")
    print(f"[worker] connecting UGV {IP}:{UGV_PORT} name={UGV_NAME}")

    uav = airsim.MultirotorClient(ip=IP, port=UAV_PORT)
    ugv = airsim.CarClient(ip=IP, port=UGV_PORT)

    uav.confirmConnection()
    ugv.confirmConnection()

    uav.enableApiControl(True, vehicle_name=UAV_NAME)
    uav.armDisarm(True, vehicle_name=UAV_NAME)

    ugv.enableApiControl(True, vehicle_name=UGV_NAME)

    print("[worker] ready")

    while True:
        vehicle, op, args, resp_q = jobs.get()
        try:
            if vehicle == UAV_NAME:
                out = handle_uav(uav, op, args)
            elif vehicle == UGV_NAME:
                out = handle_ugv(ugv, op, args)
            else:
                out = {"error": f"unknown vehicle '{vehicle}'"}

            resp_q.put(out)
        except Exception as e:
            resp_q.put({"error": str(e)})

def handle_uav(uav, op, args):
    if op == "takeoff":
        print("[uav] takeoff...")
        uav.takeoffAsync(vehicle_name=UAV_NAME).join()
        print("[uav] takeoff done")
        return {"msg": "uav takeoff ok"}

    if op == "moveToZ":
        z = float(args.get("z", -2))
        v = float(args.get("v", 1.0))
        print(f"[uav] moveToZ z={z} v={v}...")
        uav.moveToZAsync(z, v, vehicle_name=UAV_NAME).join()
        print("[uav] moveToZ done")
        return {"msg": f"uav moveToZ {z} ok"}

    if op == "hover":
        print("[uav] hover...")
        uav.hoverAsync(vehicle_name=UAV_NAME).join()
        print("[uav] hover done")
        return {"msg": "uav hover ok"}

    if op == "state":
        st = uav.getMultirotorState(vehicle_name=UAV_NAME)
        p = st.kinematics_estimated.position
        v = st.kinematics_estimated.linear_velocity
        return {
            "pos": {"x": p.x_val, "y": p.y_val, "z": p.z_val},
            "vel": {"x": v.x_val, "y": v.y_val, "z": v.z_val},
        }

    if op == "land":
        print("[uav] land...")
        uav.landAsync(vehicle_name=UAV_NAME).join()
        print("[uav] land done")
        return {"msg": "uav land ok"}

    return {"error": f"unknown uav op '{op}'"}

def handle_ugv(ugv, op, args):
    if op == "drive":
        throttle = float(args.get("throttle", 0.5))
        steering = float(args.get("steering", 0.0))
        duration = float(args.get("duration", 2.0))

        c = airsim.CarControls()
        c.throttle = throttle
        c.steering = steering
        c.brake = 0.0
        c.handbrake = False
        c.is_manual_gear = False

        # --- instrumentation like test_ugv_direct.py ---
        pose0 = ugv.simGetVehiclePose(vehicle_name=UGV_NAME)
        x0, y0, z0 = pose0.position.x_val, pose0.position.y_val, pose0.position.z_val

        print(f"[ugv] drive start thr={throttle} steer={steering} dur={duration}")
        print(f"[ugv] pose0 x={x0:.3f} y={y0:.3f} z={z0:.3f}")

        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < duration:
            ugv.setCarControls(c, vehicle_name=UGV_NAME)

            now = time.time()
            if now - last_log > 0.3:
                last_log = now
                st = ugv.getCarState(vehicle_name=UGV_NAME)
                pose = ugv.simGetVehiclePose(vehicle_name=UGV_NAME)
                dx = pose.position.x_val - x0
                dy = pose.position.y_val - y0
                dist = (dx*dx + dy*dy) ** 0.5
                print(f"[ugv] speed={st.speed:.3f} gear={st.gear} rpm={st.rpm:.1f} |Δpose|={dist:.3f}")

            time.sleep(0.05)

        pose1 = ugv.simGetVehiclePose(vehicle_name=UGV_NAME)
        x1, y1, z1 = pose1.position.x_val, pose1.position.y_val, pose1.position.z_val
        dx, dy = x1 - x0, y1 - y0
        dist = (dx*dx + dy*dy) ** 0.5
        print(f"[ugv] pose1 x={x1:.3f} y={y1:.3f} z={z1:.3f} |Δpose|={dist:.3f}")

        return {"msg": "ugv drive ok", "moved_m": dist}


    if op == "stop":
        c = airsim.CarControls()
        c.throttle = 0.0
        c.brake = 1.0
        ugv.setCarControls(c, vehicle_name=UGV_NAME)
        print("[ugv] stop")
        return {"msg": "ugv stop ok"}

    if op == "state":
        st = ugv.getCarState(vehicle_name=UGV_NAME)
        return {"speed": st.speed, "gear": st.gear, "rpm": st.rpm}

    return {"error": f"unknown ugv op '{op}'"}

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = self.client_address
        print(f"[conn] from {peer}")
        self.wfile.write(make_reply(ok=True, msg="orchestrator ready"))
        self.wfile.flush()

        while True:
            line = self.rfile.readline()
            if not line:
                print(f"[conn] closed {peer}")
                break

            try:
                req = json.loads(line.decode("utf-8"))
                vehicle = req.get("vehicle")
                op = req.get("op")
                args = req.get("args", {}) or {}

                print(f"[req] vehicle={vehicle} op={op} args={args}")

                resp_q = queue.Queue(maxsize=1)
                jobs.put((vehicle, op, args, resp_q))

                out = resp_q.get(timeout=120)  # allow long ops
                self.wfile.write(make_reply(ok=("error" not in out), **out))
                self.wfile.flush()

            except queue.Empty:
                self.wfile.write(make_reply(ok=False, error="timeout waiting for AirSim worker"))
                self.wfile.flush()
            except Exception as e:
                self.wfile.write(make_reply(ok=False, error=str(e)))
                self.wfile.flush()

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

if __name__ == "__main__":
    t = threading.Thread(target=airsim_worker, daemon=True)
    t.start()

    print(f"[server] orchestrator listening on {HOST}:{PORT}")
    with ThreadedTCPServer((HOST, PORT), Handler) as srv:
        srv.serve_forever()
