import numpy as np
from rplidar import RPLidar
import serial
from enum import IntEnum

class Instruction(IntEnum):
    STOP = 0
    FORWARD = 1
    BACKWARDS = 2
    ROTATE = 3

# declare serial communication
# Note: timeout dictates how long awaitResponse() waits
arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=5)

def get_scan():
    # connect to lidar
    lidar = RPLidar('/dev/ttyUSB0', baudrate=115200)
    # get a single scan from lidar
    for scan in lidar.iter_scans(max_buf_meas=500):
        break
    # cleanly disconnect lidar
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    # return scan
    return scan

def get_farthest(scan):
    # turn the array into an np array
    scan = np.array(scan)
    # set initial farthest value
    farthest = scan[0]
    # iterate to find farthest point with a scan quality over 13
    for point in scan:
        if point[2] > farthest[2] and point[0]> 13:
            farthest = point
    # return farthest point
    return farthest

def send_to_arduino(instruction, value):
    # write information to the arduino
    # arduino expects an int seperated by any char followed by up to 3 more ints
    # wrapped in square brackets
    arduino.write(bytes(f"[{int(instruction)},{int(value)}]", 'utf-8'))

# a simple function to test communication between arduino and pi
def wander():
    wandering = True
    while wandering:
        scan = get_scan()
        farthest = get_farthest(scan)
        print(farthest)
        send_to_arduino(Instruction.BACKWARDS, farthest[1])
        awaitResponse()
        if(input('Exit?') == 'y'):
            break

def awaitResponse():
    # read until instruction complete char is received
    while arduino.read() != b'!':
        pass
    
input("Start: ")
wander()
print("Done")
