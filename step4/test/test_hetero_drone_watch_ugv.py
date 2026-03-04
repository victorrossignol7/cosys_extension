import time
import warnings

# Évite le warning "coroutine ... was never awaited" (non bloquant mais chiant)
warnings.filterwarnings("ignore", category=RuntimeWarning)

try:
    import cosysairsim as airsim
except ImportError:
    import airsim


IP = "127.0.0.1"
DRONE_PORT = 41451
UGV_PORT   = 41452

DRONE = "Drone1"
UGV   = "UGV1"

LIFT_M = 0.35        # "décolle à peine"
DRONE_SPEED_Z = 0.5  # montée douce
FWD_THROTTLE = 0.60
FWD_TIME_S   = 4.0

TURN_THROTTLE = 0.22   # faible pour tourner quasi sur place
TURN_STEER    = 1.0
TURN_TIME_S   = 2.4    # ajuste 2.0–3.0 selon ton Husky


def ugv_apply(ugv_client, throttle, steer, brake=0.0, manual_gear=1):
    c = airsim.CarControls()
    c.is_manual_gear = True
    c.manual_gear = manual_gear
    c.throttle = float(throttle)
    c.steering = float(steer)
    c.brake = float(brake)
    c.handbrake = False
    ugv_client.setCarControls(c, vehicle_name=UGV)


def main():
    print("=== HETERO MIN TEST (LOW DRONE + UGV MOVE) ===")
    print(f"Drone: {IP}:{DRONE_PORT} name={DRONE}")
    print(f"UGV  : {IP}:{UGV_PORT} name={UGV}")

    drone = airsim.MultirotorClient(IP, port=DRONE_PORT)
    ugv   = airsim.CarClient(IP, port=UGV_PORT)

    drone.confirmConnection()
    ugv.confirmConnection()

    drone.enableApiControl(True, vehicle_name=DRONE)
    drone.armDisarm(True, vehicle_name=DRONE)

    ugv.enableApiControl(True, vehicle_name=UGV)

    # --- Drone: takeoff + petite montée + hover
    print("[drone] takeoff...")
    drone.takeoffAsync(vehicle_name=DRONE).join()

    z0 = drone.getMultirotorState(vehicle_name=DRONE).kinematics_estimated.position.z_val
    target_z = z0 - LIFT_M  # NED: monter => z diminue
    print(f"[drone] z0={z0:.3f} -> target_z={target_z:.3f} (lift={LIFT_M}m)")
    drone.moveToZAsync(target_z, DRONE_SPEED_Z, vehicle_name=DRONE).join()
    drone.hoverAsync(vehicle_name=DRONE).join()
    print("[drone] hovering low")

    # --- UGV: avance
    print("[ugv] forward...")
    ugv_apply(ugv, throttle=FWD_THROTTLE, steer=0.0, brake=0.0, manual_gear=1)
    time.sleep(FWD_TIME_S)

    # stop
    ugv_apply(ugv, throttle=0.0, steer=0.0, brake=1.0, manual_gear=1)
    time.sleep(0.5)
    ugv_apply(ugv, throttle=0.0, steer=0.0, brake=0.0, manual_gear=1)

    # --- demi-tour (tourne presque sur place)
    print("[ugv] u-turn (spin)...")
    ugv_apply(ugv, throttle=TURN_THROTTLE, steer=TURN_STEER, brake=0.0, manual_gear=1)
    time.sleep(TURN_TIME_S)

    # stop final
    ugv_apply(ugv, throttle=0.0, steer=0.0, brake=1.0, manual_gear=1)
    time.sleep(0.5)
    ugv_apply(ugv, throttle=0.0, steer=0.0, brake=0.0, manual_gear=1)
    print("[ugv] done")

    # --- Drone: reste en hover 2s puis atterrissage
    time.sleep(2.0)
    print("[drone] landing...")
    drone.landAsync(vehicle_name=DRONE).join()
    drone.armDisarm(False, vehicle_name=DRONE)
    drone.enableApiControl(False, vehicle_name=DRONE)
    ugv.enableApiControl(False, vehicle_name=UGV)

    print("=== END ===")


if __name__ == "__main__":
    main()
