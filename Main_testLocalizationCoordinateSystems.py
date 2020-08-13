from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle

from input_forDrone import droneArm, droneTakeOff, condition_yaw, send_global_velocity
from drone_variousMovements import moveDroneInLocalCoord

from dronekit import VehicleMode

import time

## Initialize connection to drone
vehicle = InitializeConnectDrone()

print("Arm the drone...")
## Arm drone
droneArm(vehicle)

print("Drone take off...")
## Drone take off - test take of to 10 meters
droneTakeOff(10, vehicle)

try:
    while 1:
##        print(send_ned_velocity_heading(movementVector[0], movementVector[1], movementVector[2], vehicle))
        print(vehicle.location.local_frame)

        time.sleep(2)

except KeyboardInterrupt:
    print("interrupted")


print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

print ("Close vehicle object")
vehicle.close()

##sitl.stop()

print("Completed")
