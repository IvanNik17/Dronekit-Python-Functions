from outputs_fromDrone import InitializeConnectDrone, ReturnDroneAngle

from input_forDrone import droneArm, droneTakeOff, condition_yaw, send_global_velocity, send_ned_velocity_heading, goto_position_target_local_ned
from drone_variousMovements import fixDistanceToBlade,moveToNewLocation, moveDroneInLocalCoord, moveToNewDistance

import time
import math
import numpy
from dronekit import VehicleMode

import serial

import json

safeDistance = 2000

distDroneToBlade = 2500



## Initialize connection to drone
vehicle = InitializeConnectDrone()

print("Arm the drone...")
## Arm drone
droneArm(vehicle)

print("Drone take off...")
## Drone take off - test take of to 10 meters
droneTakeOff(10, vehicle)

##moveToNewDistance(vehicle, 331,320)
##
##time.sleep(5)
##
##print("slept")

## move drone using velocity from given distance
def movementToDistancePosition(distX, distY):
    stopper = False

    ## x is +forward, -backward, y is +right, -left 


    signX = numpy.sign(distX)
    signY = numpy.sign(distY)
    counterX = abs(distX)
    counterY = abs(distY)
    while stopper is False:

        
        
        if counterX > 0:
            xMove = 1
            counterX-=0.5
        else:
            xMove = 0

        if counterY > 0:
            yMove = 1
            counterY-=0.5
        else:
            yMove = 0

        if yMove == 0 and xMove == 0:
            stopper = True
            
        moveDroneInLocalCoord(vehicle, [signX*xMove,signY*yMove,0], [0.5,0.5,0.5])
        time.sleep(0.1)

## move drone using velocity and given position
def movementToPositionNEW(posX, posY):
    stopper = False
    
    ## x is +forward, -backward, y is +right, -left 
    distX = posX - vehicle.location.local_frame.north
    distY = posY - vehicle.location.local_frame.east

    signX = numpy.sign(distX)
    signY = numpy.sign(distY)
    
    while True:
        moveDroneInLocalCoord(vehicle, [distX,distY,0], [1,1,1])
        if math.fabs(vehicle.location.local_frame.north) >= math.fabs(posX*0.95) and math.fabs(vehicle.location.local_frame.east) >= math.fabs(posY*0.95):
            print("Reached")
            break
        
##        time.sleep(1)

def getDataFromServer(sock):

    if sock.inWaiting():
        
        receiveMsg = sock.readline()
        
        try:
            
            unpickledDataFull = json.loads(receiveMsg.strip().decode())
                
            outputArr = numpy.array(unpickledDataFull)
                
                
            sock.write('4Send'.encode())
            
#            print(unpickledDataFull)
            
            return outputArr
        except json.JSONDecodeError:
            sock.write('4Send'.encode())
            print("error")
                
    return -1            


serialReceive = serial.Serial('COM15', 115200, timeout = 10)

print("start waiting for data...")
try:
    while True:
        sendData = getDataFromServer(serialReceive)

        if sendData is not -1:

            for i in range(0,len(sendData)):
                print(sendData[i,:])

                xPosNew = float(sendData[i,1])

                yPosNew = float(sendData[i,0])
##                while True:
                    
##                    send_ned_velocity_heading(1/sendData[i,1], 1/sendData[i,0], 0, vehicle)
                    
                        
                while True:
                        
                        distToTravel = math.sqrt((xPosNew - float(vehicle.location.local_frame.north))**2 + (yPosNew - float(vehicle.location.local_frame.east))**2)
                        
                        velX = (0.5/distToTravel)*(xPosNew - float(vehicle.location.local_frame.north))
                        velY = (0.5/distToTravel)*(yPosNew - float(vehicle.location.local_frame.east))
                        send_ned_velocity_heading(velX, velY, 0, vehicle)
                        print(vehicle.location.local_frame.north)
                        if math.fabs(float(vehicle.location.local_frame.north)) >= math.fabs(xPosNew*0.95) and math.fabs(float(vehicle.location.local_frame.east)) >= math.fabs(yPosNew*0.95):
                            print("Reached")
                            break
                        time.sleep(1)
##                movementToDistancePosition(sendData[i,1], sendData[i,0])
##                time.sleep(1)
            
except KeyboardInterrupt:
    print("exit")




##movementToDistancePosition(-1, -5)
##
##time.sleep(1)
##
##movementToDistancePosition(1, -4)
##
##time.sleep(1)
##
##movementToDistancePosition(3, -2)
##
##time.sleep(1)
##
##movementToDistancePosition(5, 0)
##
##time.sleep(1)
##
##movementToDistancePosition(4, 2)
##
##time.sleep(1)


##Rotate the drone and add the angle to the heading of the drone
##print("rotate the drone ")
##print(math.degrees(vehicle.attitude.yaw))
##condition_yaw(45,vehicle,relative=True)
##
##send_global_velocity(0,0,0,vehicle)
##time.sleep(1)
##
##print(math.degrees(vehicle.attitude.yaw))

##time.sleep(1)
##
#### Move Drone in local Coord system
##for i in range(0,10):
##    moveDroneInLocalCoord(vehicle, [1,0,0], [1,0,0])
##
##    time.sleep(0.1)
##
##print(math.degrees(vehicle.attitude.yaw))

    

## Send command to drone
##print("Fly by velocity")
##for i in range(0,10):
##    moveDroneInLocalCoord(vehicle, [1,1,0], [2,2,2])

####    fixDistanceToBlade(safeDistance, distDroneToBlade, vehicle)
####    distDroneToBlade -=10
##    ##condition_yaw(meanAngle_compensated)
##    time.sleep(1)







print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

print ("Close vehicle object")
vehicle.close()

##sitl.stop()

print("Completed")
