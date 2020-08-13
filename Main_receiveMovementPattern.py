import serial
import numpy
import json
import math
import time
from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle

from input_forDrone import droneArm, droneTakeOff, condition_yaw, send_global_velocity, send_ned_velocity_heading, goto_position_target_local_ned
from drone_variousMovements import moveDroneInLocalCoord

from dronekit import VehicleMode


def getDataFromGround(sock):
    if sock.inWaiting():
        receiveMsg = sock.readline()

##        try:
        unpickledDataFull = json.loads(receiveMsg.strip().decode())
        outputArr = numpy.array(unpickledDataFull)

        return outputArr

##        except json.JSONDecodeError:
##            print("error")

    return -1


## Initialize connection to drone
vehicle = InitializeConnectDrone()

print("Arm the drone...")
## Arm drone
droneArm(vehicle)

print("Drone take off...")
## Drone take off - test take of to 10 meters
droneTakeOff(10, vehicle)




serialReceive = serial.Serial('COM15', 115200, timeout = 10)

try:
    while True:
        sendData = getDataFromGround(serialReceive)
        if sendData is not -1:
            for i in range(0,len(sendData)):
                print(sendData[i,:]/1000)


                xPosNew = float(sendData[i,1])/1000

                yPosNew = float(sendData[i,0])/1000
                zPosNew = float(sendData[i,2])/1000
    ##                while True:
                    
    ##                    send_ned_velocity_heading(1/sendData[i,1], 1/sendData[i,0], 0, vehicle)
                    
                        
                while True:

                        dx = xPosNew - float(vehicle.location.local_frame.north)
                        dy = yPosNew - float(vehicle.location.local_frame.east)
                        dz = -zPosNew - float(vehicle.location.local_frame.down)

                        d = math.sqrt(dx*dx + dy*dy + dz*dz)

                        vx = dx/d * 0.5
                        vy = dy/d * 0.5
                        vz = dz/d * 0.5
                        
##                        distToTravel = math.sqrt((xPosNew - float(vehicle.location.local_frame.north))**2 + (yPosNew - float(vehicle.location.local_frame.east))**2)
##                        
##                        velX = (0.5/distToTravel)*(xPosNew - float(vehicle.location.local_frame.north))
##                        velY = (0.5/distToTravel)*(yPosNew - float(vehicle.location.local_frame.east))
                        send_ned_velocity_heading(vx, vy, vz, vehicle)
                        print(vehicle.location.local_frame.north)
                        if (math.fabs(float(vehicle.location.local_frame.north)) >= math.fabs(xPosNew*0.95) and math.fabs(float(vehicle.location.local_frame.east)) >= math.fabs(yPosNew*0.95) and
                            math.fabs(float(vehicle.location.local_frame.down)) >= math.fabs(zPosNew*0.95)):
                            print("Reached")
                            break
                        time.sleep(0.5)

##            movementToDistancePosition(sendData[i,1], sendData[i,0])
except KeyboardInterrupt:
    print("Exiting")
        
serialReceive.close()
print("Setting LAND mode...")
vehicle.mode = VehicleMode("RTL")

print ("Close vehicle object")
vehicle.close()

##sitl.stop()

print("Completed")
