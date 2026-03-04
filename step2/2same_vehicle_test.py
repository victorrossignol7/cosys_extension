import time
import cosysairsim as airsim

drones = ["Drone1", "Drone2"]

client = airsim.MultirotorClient()
client.confirmConnection()
print("connected")

# Enable control + arm both
for v in drones:
    client.enableApiControl(True, v)
    client.armDisarm(True, v)

# Takeoff both (parallel)
tasks = [client.takeoffAsync(vehicle_name=v) for v in drones]
[t.join() for t in tasks]
time.sleep(0.5)

# Move each drone to a different altitude to show they are distinct
client.moveToZAsync(-2.0, 1.5, vehicle_name="Drone1").join()
client.moveToZAsync(-3.0, 1.5, vehicle_name="Drone2").join()
time.sleep(0.5)

# Log 6 lines: both states each iteration
for i in range(6):
    for v in drones:
        s = client.getMultirotorState(vehicle_name=v)
        p = s.kinematics_estimated.position
        alt = -p.z_val
        print(f"{i:02d} | {v} | pos=({p.x_val:+.2f},{p.y_val:+.2f},{p.z_val:+.2f}) | alt≈{alt:.2f} m")
    # Different forward speeds to make trajectories diverge
    client.moveByVelocityZAsync(1.0, 0.0, -2.0, 0.7, vehicle_name="Drone1").join()
    client.moveByVelocityZAsync(0.5, 0.5, -3.0, 0.7, vehicle_name="Drone2").join()
    time.sleep(0.2)

# Land both (parallel)
lands = [client.landAsync(vehicle_name=v) for v in drones]
[l.join() for l in lands]

for v in drones:
    client.armDisarm(False, v)
    client.enableApiControl(False, v)

print("done")
