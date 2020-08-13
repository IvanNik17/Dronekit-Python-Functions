from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle

from input_forDrone import droneArm, droneTakeOff, condition_yaw
from drone_variousMovements import fixDistanceToBlade
from IOFunctions import waitForResponse, sendDataToGround


import serial
import time
from dronekit import VehicleMode

safeDistance = 2000

distDroneToBlade = 2500

str_return = ''

serial_IO = serial.Serial('COM10', 115200,timeout=1)

## Initialize connection to drone
vehicle = InitializeConnectDrone()

print("Arm the drone...")
## Arm drone
droneArm(vehicle)

print("Drone take off...")
## Drone take off - test take of to 10 meters
droneTakeOff(10, vehicle)

previousPos = [vehicle.location.local_frame.x,vehicle.location.local_frame.y]
currentPos = [vehicle.location.local_frame.x,vehicle.location.local_frame.y]
## Send command to drone
print("Fly by velocity")

try:
    while True:
        str_return = waitForResponse(serial_IO)

        if len(str_return) in not 0:
            
            distDroneToBlade = float(str_return)
            fixDistanceToBlade(safeDistance, distDroneToBlade, vehicle)
            currentPos = [vehicle.location.local_frame.x,vehicle.location.local_frame.y]
            

            serial_IO.write()
            
            previousPos = currentPos

except KeyboardInterrupt:
    print("Stopping")

##for i in range(0,100):
##    
##    fixDistanceToBlade(safeDistance, distDroneToBlade, vehicle)
##    distDroneToBlade -=10
##    ##condition_yaw(meanAngle_compensated)
##    time.sleep(0.001)


print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

print ("Close vehicle object")
vehicle.close()

##sitl.stop()

print("Completed")
