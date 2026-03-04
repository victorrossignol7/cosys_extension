#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uav_only_smoketest.py

- Connects to AirSim (Heterogeneous mode)
- Confirms both vehicles exist: Drone1 (multirotor) and UGV1 (ground)
- Moves ONLY the drone in several directions at fixed altitude
"""

import time
import math
import cosysairsim as airsim


HOST = "127.0.0.1"
PORT = 41451

UAV_NAME = "Drone1"
UGV_NAME = "UGV1"

ALT_M = 1.0          # positive meters, NED uses negative Z
SPEED_MPS = 2.0
LEG_S = 2.0
HOVER_S = 1.0


def require_vehicle(names, wanted):
    if wanted not in names:
        raise RuntimeError(f"Vehicle '{wanted}' not found. Available: {names}")


def main():
    client = airsim.MultirotorClient(ip=HOST, port=PORT)
    client.confirmConnection()

    # Identify vehicles
    vehicles = client.listVehicles()
    print("[ok] listVehicles ->", vehicles)

    require_vehicle(vehicles, UAV_NAME)
    require_vehicle(vehicles, UGV_NAME)
    print(f"[ok] found UAV={UAV_NAME} and UGV={UGV_NAME} (UGV will NOT move)")

    # Safety: ensure API control for UAV only
    client.enableApiControl(True, vehicle_name=UAV_NAME)
    client.armDisarm(True, vehicle_name=UAV_NAME)

    # Takeoff and go to target altitude (NED: z is negative to go up)
    print("[uav] takeoff...")
    client.takeoffAsync(vehicle_name=UAV_NAME).join()

    z = -abs(ALT_M)
    print(f"[uav] go to z={z:.2f} (NED)")
    client.moveToZAsync(z, velocity=1.5, vehicle_name=UAV_NAME).join()
    time.sleep(HOVER_S)

    # Directions in NED (x forward, y right, z down)
    # Each leg: moveByVelocityZ(vx, vy, z, duration)
    legs = [
        ("forward",        +SPEED_MPS,  0.0),
        ("backward",       -SPEED_MPS,  0.0),
        ("right",           0.0,       +SPEED_MPS),
        ("left",            0.0,       -SPEED_MPS),
        ("diag fwd-right", +SPEED_MPS / math.sqrt(2), +SPEED_MPS / math.sqrt(2)),
        ("diag fwd-left",  +SPEED_MPS / math.sqrt(2), -SPEED_MPS / math.sqrt(2)),
        ("diag back-right",-SPEED_MPS / math.sqrt(2), +SPEED_MPS / math.sqrt(2)),
        ("diag back-left", -SPEED_MPS / math.sqrt(2), -SPEED_MPS / math.sqrt(2)),
    ]

    for name, vx, vy in legs:
        print(f"[uav] {name}: vx={vx:.2f} vy={vy:.2f} for {LEG_S:.1f}s")
        client.moveByVelocityZAsync(
            vx=vx,
            vy=vy,
            z=z,
            duration=LEG_S,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
            vehicle_name=UAV_NAME
        ).join()
        time.sleep(HOVER_S)

    print("[uav] hover...")
    client.hoverAsync(vehicle_name=UAV_NAME).join()
    time.sleep(1.0)

    print("[uav] land...")
    client.landAsync(vehicle_name=UAV_NAME).join()

    client.armDisarm(False, vehicle_name=UAV_NAME)
    client.enableApiControl(False, vehicle_name=UAV_NAME)
    print("[done]")


if __name__ == "__main__":
    main()
