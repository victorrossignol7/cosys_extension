# -*- coding: utf-8 -*-
import os
import time
import math
import cosysairsim as airsim

IP = os.environ.get("AIRSIM_IP", "127.0.0.1")
UGV_PORT = int(os.environ.get("UGV_PORT", "41452"))
UGV_NAME = os.environ.get("UGV_NAME", "UGV1")

def dist3(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def pose_xyz(pose):
    p = pose.position
    return (p.x_val, p.y_val, p.z_val)

def print_state(tag, st, pose, d):
    xyz = pose_xyz(pose)
    print(f"{tag} speed={st.speed:.3f} gear={st.gear} rpm={st.rpm:.3f}  |Δpose|={d:.4f}  pose={xyz}")

def run_phase(c, ctrl, xyz0, phase_name, duration_s=2.5, step_s=0.2):
    print(f"\n=== PHASE: {phase_name} ===")
    t0 = time.time()
    moved = 0
    last_pose = None

    while time.time() - t0 < duration_s:
        c.setCarControls(ctrl, vehicle_name=UGV_NAME)

        st = c.getCarState(vehicle_name=UGV_NAME)
        pose = c.simGetVehiclePose(vehicle_name=UGV_NAME)
        xyz = pose_xyz(pose)
        d = dist3(xyz, xyz0)

        print_state("[run] ", st, pose, d)

        if d > 0.02:
            moved += 1

        last_pose = pose
        time.sleep(step_s)

    return moved, last_pose

def main():
    print("=== UGV DIAG START (airsim only) ===")
    print(f"Target: {IP}:{UGV_PORT} vehicle='{UGV_NAME}'")

    c = airsim.CarClient(ip=IP, port=UGV_PORT)
    c.confirmConnection()
    print("Connected!")

    # API control ON
    c.enableApiControl(True, vehicle_name=UGV_NAME)
    print("[probe] enableApiControl(True) sent")

    st0 = c.getCarState(vehicle_name=UGV_NAME)
    p0 = c.simGetVehiclePose(vehicle_name=UGV_NAME)
    xyz0 = pose_xyz(p0)
    print(f"[init] speed={st0.speed} gear={st0.gear} rpm={st0.rpm}")
    print(f"[init] pose={xyz0}  (note: Z proche de 0 attendu pour un UGV)")

    # --- Phase 1: Auto gears, forward
    ctrl = airsim.CarControls()
    ctrl.throttle = 0.6
    ctrl.steering = 0.0
    ctrl.brake = 0.0
    ctrl.handbrake = False
    ctrl.is_manual_gear = False
    ctrl.manual_gear = 0

    moved1, _ = run_phase(c, ctrl, xyz0, "AUTO GEARS forward")

    # --- Phase 2: Manual gear = 1, forward
    ctrl2 = airsim.CarControls()
    ctrl2.throttle = 0.6
    ctrl2.steering = 0.0
    ctrl2.brake = 0.0
    ctrl2.handbrake = False
    ctrl2.is_manual_gear = True
    ctrl2.manual_gear = 1

    moved2, _ = run_phase(c, ctrl2, xyz0, "MANUAL gear=1 forward")

    # --- Phase 3: Manual gear = -1, reverse (petit throttle)
    ctrl3 = airsim.CarControls()
    ctrl3.throttle = 0.35
    ctrl3.steering = 0.0
    ctrl3.brake = 0.0
    ctrl3.handbrake = False
    ctrl3.is_manual_gear = True
    ctrl3.manual_gear = -1

    moved3, _ = run_phase(c, ctrl3, xyz0, "MANUAL gear=-1 reverse")

    # Stop
    stop = airsim.CarControls()
    stop.throttle = 0.0
    stop.steering = 0.0
    stop.brake = 0.5
    stop.handbrake = False
    stop.is_manual_gear = False
    stop.manual_gear = 0
    c.setCarControls(stop, vehicle_name=UGV_NAME)
    time.sleep(0.2)

    st_end = c.getCarState(vehicle_name=UGV_NAME)
    p_end = c.simGetVehiclePose(vehicle_name=UGV_NAME)
    xyz_end = pose_xyz(p_end)
    total_d = dist3(xyz_end, xyz0)

    print("\n=== SUMMARY ===")
    print(f"moved_samples: auto={moved1} manual+1={moved2} manual-1={moved3}")
    print(f"[end] speed={st_end.speed:.3f} gear={st_end.gear} rpm={st_end.rpm:.3f}")
    print(f"[result] total |Δpose|={total_d:.4f} start_pose={xyz0} end_pose={xyz_end}")

    if total_d > 0.05:
        print("[diag] ✅ UGV bouge : commande + mouvement OK.")
    else:
        print("[diag] ❌ UGV ne bouge pas : commande envoyée mais movement/physics ne s’applique pas.")
        print("[diag] Interprétation rapide : si MANUAL gear=1 ne bouge pas non plus -> le blocage est côté ChaosVehicle/possess/controller/park/inputs.")
        print("[diag] NEXT STEP: vérifier côté C++ (SkidVehiclePawnApi) : requiresControllerForInputs, parked, override controller, et que les inputs arrivent au MovementComponent.")

    print("=== UGV DIAG END ===")

if __name__ == "__main__":
    main()
