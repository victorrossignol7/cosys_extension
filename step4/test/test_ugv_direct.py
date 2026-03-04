# -*- coding: utf-8 -*-
"""
test_ugv_direct.py
Test direct UGV (Car RPC sur port 41452 par défaut) dans un SimMode Heterogeneous.

Objectif:
- Vérifier que le client parle bien au bon serveur (UGV:41452)
- Envoyer des CarControls en boucle
- Logger speed/gear/rpm + delta pose (preuve de mouvement)
"""

import os
import time
import math
import argparse
import cosysairsim as airsim


def pose_xyz(p):
    return (p.position.x_val, p.position.y_val, p.position.z_val)


def dist3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=os.environ.get("AIRSIM_IP", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_PORT", "41452")))
    parser.add_argument("--name", default=os.environ.get("UGV_NAME", "UGV1"))

    parser.add_argument("--duration", type=float, default=float(os.environ.get("DURATION", "5.0")))
    parser.add_argument("--throttle", type=float, default=float(os.environ.get("THROTTLE", "0.6")))
    parser.add_argument("--steering", type=float, default=float(os.environ.get("STEERING", "0.0")))
    parser.add_argument("--brake", type=float, default=float(os.environ.get("BRAKE", "0.0")))
    parser.add_argument("--handbrake", action="store_true", default=os.environ.get("HANDBRAKE", "0") == "1")

    # Gear options
    # By default: auto gears (like your orchestrator)
    parser.add_argument("--manual", action="store_true", default=os.environ.get("MANUAL_GEAR", "0") == "1")
    parser.add_argument("--gear", type=int, default=int(os.environ.get("GEAR", "1")))  # used only if --manual

    # Reset is optional; default OFF to avoid "multiple reset calls" issues
    parser.add_argument("--reset", action="store_true", default=os.environ.get("RESET", "0") == "1")

    # Logging cadence
    parser.add_argument("--send_hz", type=float, default=float(os.environ.get("SEND_HZ", "20")))
    parser.add_argument("--print_hz", type=float, default=float(os.environ.get("PRINT_HZ", "5")))

    args = parser.parse_args()

    print("=== UGV DIRECT TEST START ===")
    print(f"Target: {args.ip}:{args.port} vehicle='{args.name}'")
    print(f"Cmd: throttle={args.throttle} steering={args.steering} brake={args.brake} handbrake={int(args.handbrake)} "
          f"manual={int(args.manual)} gear={args.gear if args.manual else 'AUTO'} duration={args.duration}s")

    client = airsim.CarClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    print("Connected!")

    client.enableApiControl(True, vehicle_name=args.name)
    print("[ugv] enableApiControl(True)")

    if args.reset:
        print("[ugv] reset() ...")
        client.reset()
        time.sleep(0.2)

    # Snapshot start
    st0 = client.getCarState(vehicle_name=args.name)
    p0 = client.simGetVehiclePose(vehicle_name=args.name)
    xyz0 = pose_xyz(p0)
    print(f"[init] speed={st0.speed:.3f} gear={st0.gear} rpm={st0.rpm:.3f} pose={xyz0}")

    # Apply a short "neutral" command (helps avoid residual brake states)
    neutral = airsim.CarControls()
    neutral.throttle = 0.0
    neutral.steering = 0.0
    neutral.brake = 0.2
    neutral.handbrake = False
    neutral.is_manual_gear = False
    neutral.manual_gear = 0
    client.setCarControls(neutral, vehicle_name=args.name)
    time.sleep(0.2)

    # Main command
    ctrl = airsim.CarControls()
    ctrl.throttle = float(args.throttle)
    ctrl.steering = float(args.steering)
    ctrl.brake = float(args.brake)
    ctrl.handbrake = bool(args.handbrake)

    if args.manual:
        ctrl.is_manual_gear = True
        ctrl.manual_gear = int(args.gear)
    else:
        ctrl.is_manual_gear = False
        ctrl.manual_gear = 0

    send_dt = 1.0 / max(1.0, args.send_hz)
    print_dt = 1.0 / max(1.0, args.print_hz)

    t0 = time.time()
    t_last_print = 0.0
    moved_samples = 0

    while time.time() - t0 < args.duration:
        client.setCarControls(ctrl, vehicle_name=args.name)

        now = time.time()
        if now - t_last_print >= print_dt:
            st = client.getCarState(vehicle_name=args.name)
            p = client.simGetVehiclePose(vehicle_name=args.name)
            xyz = pose_xyz(p)
            d = dist3(xyz, xyz0)
            if d > 0.02:
                moved_samples += 1

            print(f"[run] speed={st.speed:.3f} gear={st.gear} rpm={st.rpm:.3f} |Δpose|={d:.4f} pose={xyz}")
            t_last_print = now

        time.sleep(send_dt)

    # Stop
    stop = airsim.CarControls()
    stop.throttle = 0.0
    stop.steering = 0.0
    stop.brake = 1.0
    stop.handbrake = False
    stop.is_manual_gear = False
    stop.manual_gear = 0
    client.setCarControls(stop, vehicle_name=args.name)
    time.sleep(0.2)

    st_end = client.getCarState(vehicle_name=args.name)
    p_end = client.simGetVehiclePose(vehicle_name=args.name)
    xyz_end = pose_xyz(p_end)
    total_d = dist3(xyz_end, xyz0)

    print("\n=== SUMMARY ===")
    print(f"moved_samples={moved_samples}")
    print(f"[end] speed={st_end.speed:.3f} gear={st_end.gear} rpm={st_end.rpm:.3f}")
    print(f"[result] total |Δpose|={total_d:.4f} start_pose={xyz0} end_pose={xyz_end}")

    if total_d > 0.05:
        print("[result] ✅ UGV bouge : pipeline CarRpcLibServer OK (port/vehicle_name correct).")
    else:
        print("[result] ❌ UGV ne bouge pas : vérifier port=41452 + vehicle_name='UGV1' + pas d'autre script qui envoie brake/throttle=0.")

    print("=== UGV DIRECT TEST END ===")


if __name__ == "__main__":
    main()
