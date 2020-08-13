import serial
import numpy
import json


def waitForResponse(ser):

    
    if (ser.inWaiting()>0): #if incoming bytes are waiting to be read from the serial input buffer
##            data_str = serialSend.read(serialSend.inWaiting()).decode() #read the bytes and convert from binary array to ASCII
        num_read = ser.read(1).decode()
        data_str = ser.read(int(num_read)).decode()
        return data_str


def sendDataToGround(ser,arrayData):

        arrayData = numpy.round(arrayData,1)
        listData = arrayData.tolist()

        jsonPoints = json.dumps(listData)

        sendAll = jsonPoints + '\n'

        ser.write(sendAll)
