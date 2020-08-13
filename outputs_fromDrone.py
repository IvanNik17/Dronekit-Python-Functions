from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse  


def InitializeConnectDrone():


        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14551')
        #parser.add_argument('--connect', default='/dev/serial0')
        args = parser.parse_args()

        # Connect to the Vehicle
        print 'Connecting to vehicle on: %s' % args.connect
        vehicle = connect(args.connect, baud=57600, wait_ready=True)

        vehicle.wait_ready('autopilot_version')
        return vehicle
	
	
def ReturnDroneAngle(vehicle):
        yawAngle = math.degrees( vehicle.attitude.yaw)
        heightDrone = vehicle.location.global_relative_frame.alt * 1000
        return yawAngle,heightDrone
