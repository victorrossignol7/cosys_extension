# -*- coding: utf-8 -*-

# test_uav_direct.py
# Test direct UAV (Multirotor RPC sur port 41451 par défaut)

import os
import time
import cosysairsim as airsim

AIRSIM_IP = os.environ.get("AIRSIM_IP", "127.0.0.1")
AIRSIM_PORT = int(os.environ.get("AIRSIM_PORT", "41451"))

UAV_NAME = os.environ.get("UAV_NAME", "Drone1")

uav = airsim.MultirotorClient(AIRSIM_IP, AIRSIM_PORT)
uav.confirmConnection()

uav.enableApiControl(True, UAV_NAME)
uav.armDisarm(True, UAV_NAME)

print("[uav] takeoff...")
uav.takeoffAsync(vehicle_name=UAV_NAME).join()

print("[uav] moveToZ -2...")
uav.moveToZAsync(-2, 1, vehicle_name=UAV_NAME).join()

time.sleep(1.0)

print("[uav] hover...")
uav.hoverAsync(vehicle_name=UAV_NAME).join()

time.sleep(1.0)

print("[uav] land...")
uav.landAsync(vehicle_name=UAV_NAME).join()

uav.armDisarm(False, UAV_NAME)
uav.enableApiControl(False, UAV_NAME)

print("[uav] OK")
