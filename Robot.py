import cv2 as cv
import numpy as np
import time
from rplidar import RPLidar

import serial.tools.list_ports # for communication with arduino
# pip install pyserial

leftDot = (113, 413) # where the can will fall
leftEnd = (253, 327)

rightDot = (575, 413)
rightEnd = (485, 327)

DETECTED = False

IN_RANGE = False
ON = False

TARGET_ANGLE = None
HEADING = 0
SCAN = []

class Camera:
    def __init__(self, port=0):
        self.port = port
        self.camera = cv.VideoCapture(self.port)
        self.fourcc = cv.VideoWriter_fourcc(*'XVID')
        #self.out = cv.VideoWriter('output.avi', self.fourcc, 20.0, (640, 480))
        self.frame = None
        self.h = 480
        self.w = 640

    def captureFrame(self):
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                self.frame = frame

    def getFrame(self):
        return self.frame

    def closeCamera(self):
        self.camera.release()

def drawGuide(frame):
    # left
    cv.line(frame, (0, 480), leftEnd, color =(0,0,255), thickness=2)
    cv.circle(frame, leftDot, radius = 5, color =(0,0,255), thickness=-1)

    # right
    cv.line(frame, (640, 480), rightEnd, color = (0,0,255), thickness=2)
    cv.circle(frame, rightDot, radius = 5, color =(0,0,255), thickness=-1)

    # connecting line
    cv.line(frame, leftDot, rightDot, color = (0,0,255), thickness=2)

    # end line and dots
    cv.line(frame, leftEnd, rightEnd, color = (0,0,255), thickness=2)
    cv.circle(frame, leftEnd, radius = 5, color =(0,0,255), thickness=-1)
    cv.circle(frame, rightEnd, radius = 5, color =(0,0,255), thickness=-1)

    return frame


polyWholeGuide = [(0,480), (640, 480), leftEnd, rightEnd]
polyLowerGuide = [(0,480), (640, 480), leftDot, rightDot] # used for checking if within distance of bin
# to stop and deposit can


def get_scan():
    # connect to lidar
    lidar = RPLidar('COM7', baudrate=115200)
    # get a single scan from lidar
    for scan in lidar.iter_scans(max_buf_meas=360):
        break
    # cleanly disconnect lidar
    lidar.clean_input()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    # return scan
    return scan

# more generalized form. pass in list of cords that make up the polygon
# form of polygon = [(0,0), (0,1)] etc
def checkPixelPolygon(point, polygon):
    numVert = len(polygon)

    x = point[0]
    y = point[1]

    intersects = 0

    p1 = polygon[0] # getting first point of polygon to check given point against

    for i in range(numVert): # checking against each vertex

        # next point
        p2 = polygon[(i + 1) % numVert]

        if(y > min(p1[1], p2[1]) and y <= max(p1[1], p2[1])): # between y given with current points

            if(x <= max(p1[0], p2[0])): # left of max x

                if(p1[1] != p2[1]): # not checking points on same line already
                    # calc intersection
                    xInt = (y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0]
                
                    # check if intersect is on line
                    if (x <= xInt):
                        intersects += 1
                else: # on same edge. counting as within for now, but can switch it.
                    intersects += 1
        
        p1 = p2

    if (intersects % 2 == 1): return True # odd nums
    else: return False # even


def checkPixel(point):
    # based on ray tracing
    # given point acts as a horizontal line from which intercepts with the polygon are determined.
    # even intercepts = outside, odd intercepts = inside.

    # references:
    # https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
    # https://stackoverflow.com/questions/11716268/point-in-polygon-algorithm


    polygon = [(0,480), (640, 480), leftEnd, rightEnd] # points that the polygon is composed of
    numVert = len(polygon)

    x = point[0]
    y = point[1]

    intersects = 0

    p1 = polygon[0] # getting first point of polygon to check given point against

    for i in range(numVert): # checking against each vertex

        # next point
        p2 = polygon[(i + 1) % numVert]

        if(y > min(p1[1], p2[1]) and y <= max(p1[1], p2[1])): # between y given with current points

            if(x <= max(p1[0], p2[0])): # left of max x

                if(p1[1] != p2[1]): # not checking points on same line already
                    # calc intersection
                    xInt = (y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0]
                
                    # check if intersect is on line
                    if (x <= xInt):
                        intersects += 1
                else: # on same edge. counting as within for now, but can switch it.
                    intersects += 1
        
        p1 = p2

    if (intersects % 2 == 1): return True # odd nums
    else: return False # even

def read(x):
    if x.in_waiting > 0:
        input = x.readline().decode('utf-8').strip()
        print(input)
        if input == "PLAY/PAUSE":
            return "PLAY/PAUSE"
        elif input == "STOP":
            return "STOP"
        elif input == "UP":
            return "UP"
        elif input == "ROTATE":
            return "ROTATE"
        elif input == "WANDER":
            return "WANDER"
    return False


def findColor(frame):
    blue_lower = np.array([148, 35, 16], np.uint8)
    blue_upper = np.array([190, 45, 30], np.uint8)
    mask = cv.inRange(frame, blue_lower, blue_upper)
    # Uncomment these to see where the blue is detected
    # res_blue = cv.bitwise_and(frame, frame, mask=mask)
    # cv.imshow("mask", res_blue)
    coords = cv.findNonZero(mask)
    if coords is not None:
        global DETECTED
        DETECTED = True
        global BLUE_FRAME
        BLUE_FRAME = True
        if(IN_RANGE == True):
            frame = cv.putText(frame, "Within Range", (100, 150), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        else:
            frame = cv.putText(frame, "Blue Detected", (100, 150), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
            # else:
            #     DETECTED = False
    else:
        DETECTED = False
        BLUE_FRAME = False
        
    return frame

def withinRange(frame):
    blue_lower = np.array([148, 35, 16], np.uint8)
    blue_upper = np.array([190, 45, 30], np.uint8)
    mask = cv.inRange(frame, blue_lower, blue_upper)
    coords = cv.findNonZero(mask)
    if coords is not None:
        for point in coords:
            if checkPixelPolygon((point[0][0], point[0][1]), polyLowerGuide):
                global IN_RANGE
                IN_RANGE = True
            else:
                IN_RANGE = False
    else:
        IN_RANGE = False
    return

BLUE_FRAME = False
CENTER_BIN = False

def centerBin (c, serialInst):
    if(BLUE_FRAME == True):
        left = False # x 0-320

        right = False # x 321 - 640

        while(left != True and right != True):

            blue_lower = np.array([148, 35, 16], np.uint8)
            blue_upper = np.array([190, 45, 30], np.uint8)
            mask = cv.inRange(frame, blue_lower, blue_upper)
            coords = cv.findNonZero(mask)
            if coords is not None:
                for point in coords:
                    if (point[0][0] in range(0,320)):
                        left = True
                    elif(point[0][0] in range(321, 639)):
                        right = True

            if(left == True):
                print("Bin on left")
                command = "turnLeft"
                serialInst.write(command.encode('utf-8'))
                pass # call function to turn
            elif(right == True):
                print("Bin on right")
                command = "turnRight"
                serialInst.write(command.encode('utf-8'))
                pass

            c.captureFrame()
            frame = c.getFrame()
        
            if frame is not None:
                withinRange(frame)
                cv.imshow('frame', findColor(drawGuide(frame)))

    else:
        return # blue isn't in frame; don't need to turn to center
    
    global CENTER_BIN 
    CENTER_BIN = True
    return # bin should be centered (ish) on the camera


def wander():
    global TARGET_ANGLE
    global SCAN
    global HEADING
    if len(SCAN) == 0:
        scan = get_scan()
        SCAN = get_farthest(scan)
        pass

    if not TARGET_ANGLE and not DETECTED:
        TARGET_ANGLE = int(SCAN[1]+HEADING)
        if TARGET_ANGLE > 360:
            TARGET_ANGLE -=360
    print(f"Heading:{HEADING}\nTarget:{TARGET_ANGLE}\nScan:{SCAN}\n")
    if abs(HEADING - TARGET_ANGLE) > 5 and TARGET_ANGLE:
        command = "rotate_"
        serialInst.write(command.encode('utf-8'))
        time.sleep(0.1)
        angle = serialInst.readline().decode().strip('\n\r')
        if angle:
            HEADING = int(angle)
        
    elif not DETECTED:
        scan = get_scan()
        mins = np.min(scan, axis=1)
        if mins[2] > 50:
            command = "forward_"
            serialInst.write(command.encode('utf-8'))
        else:
            command = "stop_"
            serialInst.write(command.encode('utf-8'))
            TARGET_ANGLE = None
            SCAN = []
            #move to rotate looking for recycling bin 


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

def testing(serialInst):
    input = read(serialInst)

    if input == "UP":
        command = "forward"
        serialInst.write(command.encode('utf-8'))
    elif input == "ROTATE":
        command = "rotate test"
        serialInst.write(command.encode('utf-8'))
    elif input == "STOP":
        command = "stop"
        serialInst.write(command.encode('utf-8'))
    elif input == "DUMP":
        command = "dump"
        serialInst.write(command.encode('utf-8'))
    elif input == "WANDER":
        wander()

if __name__ == "__main__":
    c = Camera(0)
    startTime = time.time()

    # establish connection to serial port
    ports = serial.tools.list_ports.comports()
    serialInst = serial.Serial(timeout=1)
    portsList = []
    
    for p in ports:
        portsList.append(str(p))
        print(str(p)) # shows available COM connections to chose from
        
    # picking COM port that arduino is on
    # won't need user input once connected with the pi, but varies from computer to computer
    # com = input("Select COM port for Arduino #: ")
    # print(com)
    
    # for i in range(len(portsList)):
    #     # ensure input is valid
    #     if portsList[i].startswith("COM" + str(com)):
    #         use = "COM" + str(com)
    #         #print("Using: " + use)
    
    # form connection
    serialInst.baudrate = 115200
    serialInst.port = "COM17"
    serialInst.open()
    time.sleep(0.05)

    # everything below can probably go into a new function
    
    currentDetectedState = False # need to check if movement has already been triggered
    
    # continuously take in info from the camera and interpret it
    # based on what is seen, perform different actions
    on=True
    while True:
        # Testing function using remote input
        # testing(serialInst)

        # if(read(serialInst)):
        #     # print(serialInst)
        #     if not on:
        #         c.captureFrame()
        #         on = True
        #     else:
        #         on = False
        #         command = "stop"
        #         serialInst.write(command.encode('utf-8'))
        #         c.closeCamera()
        #         cv.destroyAllWindows()
        #         break
        if on:
            c.captureFrame()
            frame = c.getFrame()
            
            if(time.time() - startTime >= 300): # 5 minutes
                c.closeCamera()
                cv.destroyAllWindows()
                quit()

            if frame is not None:
                withinRange(frame)
                frame = drawGuide(findColor(frame))
                cv.imshow('frame', frame)

            if(DETECTED == False and currentDetectedState != DETECTED):# detected gets checked/altered in findColour()
                command = "stop"
                serialInst.write(command.encode('utf-8'))
                currentDetectedState = DETECTED
            elif (DETECTED == True and currentDetectedState != DETECTED):
                # ensure bin is in the center of the screen before moving forward
                #if(CENTER_BIN == False):
                    #centerBin()
                command = "forward"
                serialInst.write(command.encode('utf-8'))
                currentDetectedState = DETECTED
                print(command)
            elif(IN_RANGE == True):
                # command = "stop"
                # print(command)
                # serialInst.write(command.encode('utf-8'))
                command = "dump"
                print(command)
                serialInst.write(command.encode('utf-8'))
                break
            elif (DETECTED == False):
                wander()

            # can use below code for error checking and to ensure info is passed
            #command = input("Command: ")
            #serialInst.write(command.encode('utf-8'))

            #if command == 'exit':
                #quit()

        key = cv.waitKey(1)
        if key == ord('q'):
            break

    c.closeCamera()
    cv.destroyAllWindows()
