from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle

from input_forDrone import droneArm, droneTakeOff, condition_yaw
from drone_variousMovements import fixDistanceToBlade,moveToNewLocation, moveDroneInLocalCoord

import time
import math
from dronekit import VehicleMode



## Initialize connection to drone
vehicle = InitializeConnectDrone()

print("Arm the drone...")
## Arm drone
droneArm(vehicle)


try:
    while True:
        yawAngle, heightMeasure = ReturnDroneAngle(vehicle)


        print("Yaw Angle = " + str(yawAngle) + "| Height = " + str(heightMeasure))
        print(vehicle.velocity)
##        print(vehicle.gps_0)
##        print(vehicle.location.global_frame)
except KeyboardInterrupt:
    print("Stopping")

vehicle.close()
