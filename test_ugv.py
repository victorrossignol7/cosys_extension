#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cosysairsim as airsim

HOST = "127.0.0.1"
UGV_PORT = 41452
UGV_NAME = "UGV1"

def try_call(obj, name, *args, **kwargs):
    if not hasattr(obj, name):
        return None, f"missing:{name}"
    fn = getattr(obj, name)
    try:
        return fn(*args, **kwargs), None
    except TypeError:
        # sometimes older signatures don't use keyword vehicle_name
        try:
            return fn(*args), None
        except Exception as e:
            return None, str(e)
    except Exception as e:
        return None, str(e)

def try_pose(client):
    for fn in ("simGetVehiclePose", "getVehiclePose"):
        val, err = try_call(client, fn, vehicle_name=UGV_NAME)
        if err is None and val is not None:
            return val, None
    return None, "pose_fn_missing_or_failed"

def pose_str(p):
    if p is None:
        return "pose=?"
    return f"pose=({p.position.x_val:.2f},{p.position.y_val:.2f},{p.position.z_val:.2f})"

def main():
    car = airsim.CarClient(ip=HOST, port=UGV_PORT)
    car.confirmConnection()

    # 1) Read API control flag (if available)
    api0, err0 = try_call(car, "isApiControlEnabled", vehicle_name=UGV_NAME)
    print("[before] isApiControlEnabled =", api0, "err=", err0)

    # 2) Enable API control
    _, errE = try_call(car, "enableApiControl", True, vehicle_name=UGV_NAME)
    print("[call] enableApiControl(True) err=", errE)

    api1, err1 = try_call(car, "isApiControlEnabled", vehicle_name=UGV_NAME)
    print("[after ] isApiControlEnabled =", api1, "err=", err1)

    # 3) Send a non-zero control
    c = airsim.CarControls()
    c.throttle = 0.7
    c.steering = 0.0
    c.brake = 0.0
    c.handbrake = False
    c.is_manual_gear = True
    c.manual_gear = 1
    c.gear_immediate = True

    _, errS = try_call(car, "setCarControls", c, vehicle_name=UGV_NAME)
    print("[call] setCarControls(throttle=0.7) err=", errS)

    # 4) Read back controls a few times to see if they get overwritten to 0
    for i in range(100):
        ctrl, errC = try_call(car, "getCarControls", vehicle_name=UGV_NAME)
        st, errSt = try_call(car, "getCarState", vehicle_name=UGV_NAME)
        p, _ = try_pose(car)

        if ctrl is None:
            ctrl_s = f"getCarControls=? (err={errC})"
        else:
            ctrl_s = (
                f"ctrl(thr={ctrl.throttle:.2f}, steer={ctrl.steering:.2f}, brk={ctrl.brake:.2f}, hb={int(getattr(ctrl,'handbrake',0))}, "
                f"man={int(getattr(ctrl,'is_manual_gear',0))}, g={getattr(ctrl,'manual_gear',None)})"
            )

        spd = getattr(st, "speed", None) if st is not None else None
        gear = getattr(st, "gear", None) if st is not None else None
        rpm = getattr(st, "rpm", None) if st is not None else None
        state_s = f"speed={spd:.3f} gear={gear} rpm={rpm:.1f}" if spd is not None else f"state=? (err={errSt})"

        print(f"[t{i:02d}] {pose_str(p)} {state_s} {ctrl_s}")
        time.sleep(0.2)

    print("[done]")

if __name__ == "__main__":
    main()
