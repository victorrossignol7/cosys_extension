import time
import cosysairsim as airsim

VEH = "Drone1"  # Must match the vehicle name in settings.json

# Connect to the AirSim server running inside Unreal (requires the level to be in "Play")
client = airsim.MultirotorClient()
client.confirmConnection()
print("connected")  # Screenshot this line as proof of successful connection

# Take control of the drone and arm it (motors enabled)
client.enableApiControl(True, VEH)
client.armDisarm(True, VEH)

# Take off, then stabilize at ~2 meters altitude.
# AirSim uses NED coordinates: z is DOWN, so z = -2.0 means ~2m above the origin.
client.takeoffAsync(vehicle_name=VEH).join()
time.sleep(0.5)
client.moveToZAsync(-2.0, 1.5, vehicle_name=VEH).join()
time.sleep(0.5)

# Log state + IMU (sensor) and move forward while maintaining altitude
for i in range(8):
    state = client.getMultirotorState(vehicle_name=VEH)
    p = state.kinematics_estimated.position
    alt = -p.z_val  # Convert NED z to approximate altitude (meters)

    imu = client.getImuData(vehicle_name=VEH)

    print(
        f"{i:02d} | pos=({p.x_val:+.2f},{p.y_val:+.2f},{p.z_val:+.2f}) "
        f"| alt≈{alt:.2f} m "
        f"| imu_w=({imu.angular_velocity.x_val:+.2f},{imu.angular_velocity.y_val:+.2f},{imu.angular_velocity.z_val:+.2f})"
    )

    # Command forward velocity while holding altitude at z=-2.0 (NED)
    client.moveByVelocityZAsync(
        vx=1.0,
        vy=0.0,
        z=-2.0,
        duration=0.5,
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
        vehicle_name=VEH,
    ).join()
    time.sleep(0.2)

# Land and release control cleanly
client.landAsync(vehicle_name=VEH).join()
client.armDisarm(False, VEH)
client.enableApiControl(False, VEH)

print("done")
