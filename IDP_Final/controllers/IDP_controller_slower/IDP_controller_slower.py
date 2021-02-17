from controller import Robot
import numpy as np
from queue import *
import struct
np.set_printoptions(threshold=np.inf, linewidth=np.inf)
###Set up parameters and sensors###

TIME_STEP = 64
robot = Robot()

# Distance sensors, return value can be matched to distance with a function
ds_long = robot.getDevice('Sharp IR sensor GP2Y0A02YK0F')
ds_long.enable(TIME_STEP)

ds_short = robot.getDevice('Sharp IR sensor GP2D120')
ds_short.enable(TIME_STEP)

# Motors on each wheel 
wheels = []
wheelsNames = ['wheel1', 'wheel2']

for i in range(len(wheelsNames)):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# Motor on Grabber
grabber = robot.getDevice("grabber")
grabber.setPosition(float('inf'))
grabber.setVelocity(0.0)

# Compass returning cartesian vector of north direction relative to robot bearing
cp = robot.getDevice("compass")
cp.enable(TIME_STEP)

# GPS returning cartesian vector of robot global position
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# Camera returning the rgb value of detected object
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

#emitter used to send data
emitter = robot.getDevice("emitter_red")
emitter.setChannel(1)
emitter.setRange(-1)

#Receiver used to send data
receiver = robot.getDevice("receiver_red")
receiver.enable(TIME_STEP)
receiver.setChannel(1)

###Global variables###

robotWidth = 0.16
robotLength = 0.2
blockSize = 0.05
floorSize = 2.4
mapSize = round(floorSize / blockSize)

# Represent map by a 48x48 matrix of grid size 0.05m, 1 if obstacle exists, otherwise 0, padded with ones to represent wall
def Init_MapMat():
    global mapMat
    mapMat = np.pad(np.zeros((mapSize - 6,mapSize - 6)), 3, 'constant', constant_values=1)
    return mapMat

mapMat = Init_MapMat()

# List of the center global position (X,Z) of all blocks in the playground, with collected blocks removed during runtime
blockList = []
toGrabList = []
blocksCollected = 0
# Global position (X,Z) of home
home = (0.95,0.95)
# Global position (X,Z) of current target
target = (0,0)
lastGrabPosition = (0,0)

path = []
curIndex = 0
currentState = 0
grabCounter = 0
IsMovingToGrab = True

init_detection_counter = 142
detection_counter = init_detection_counter
object_detected_counter = [0]

stepBackCounter = 0

IsRed = False
IsBlue = False
IsColourChecked = False

IsNewScanEmpty = False

message_to_send = b'0x00'

init_missed_counter = 20
missed_counter = init_missed_counter
IsTurningRight = True

sendListCounter = 0

###Auxiliary Functions###

def Test_Compass():
    print("Compass")
    print("X:", cp.getValues()[0])
    print("Y:", cp.getValues()[1])
    print("Z:", cp.getValues()[2])
    print("Bearing:", Get_Robot_Bearing())
    print("#####################")

def Test_GPS():
    print("GPS")
    print("X:", gps.getValues()[0])
    print("Y:", gps.getValues()[1])
    print("Z:", gps.getValues()[2])
    print("#####################")

def Pos_To_Mat_Coord(pos):
    x = round((pos[0] + floorSize/2) / blockSize)
    z = round((pos[1] + floorSize/2) / blockSize)
    matCoord = (x,z)
    return matCoord

def Mat_Coord_To_Pos(coord):
    x = coord[0] * blockSize - floorSize/2
    z = coord[1] * blockSize - floorSize/2
    pos = (x,z)
    return pos

def Steps_Between(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def Angle_Between(v1, v2):
    angle = Get_Bearing_In_Degrees(v1) - Get_Bearing_In_Degrees(v2)
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360
    return angle

def Get_Bearing_In_Degrees(vec):
    radian = np.arctan2(vec[0], vec[1])
    bearing = (radian - 1.5708) / np.pi * 180.0
    if (bearing < 0.0):
        bearing = bearing + 360.0
    return bearing

def Get_Robot_Bearing():
    return Get_Bearing_In_Degrees((cp.getValues()[0], cp.getValues()[2]))

def Get_Direction():
    theta = np.deg2rad(Get_Robot_Bearing())
    rot = -np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return np.dot(rot, [0,-1])

def Find_Index_Of_Closest_Node():
    min_dist = np.inf
    min_index = 0
    for i in range(len(path)):
        node_pos = Mat_Coord_To_Pos(path[i])
        dist = np.sqrt(pow(node_pos[0] - gps.getValues()[0], 2) + pow(node_pos[1] - gps.getValues()[2], 2))
        if dist < min_dist:
            min_dist = dist
            min_index = i
    return min_index

def Update_List(pos, isAdd):
    global blockList
    global toGrabList
    global lastGrabPosition
    if isAdd:
        blockList.append(pos)
        toGrabList.append(pos)
    else:
        blockList.remove(pos)
        toGrabList.remove(pos)
        lastGrabPosition = (pos[0],pos[1])
    blockList.sort(key=lambda tup: Steps_Between(tup,home))
    toGrabList.sort(key=lambda tup: Steps_Between(tup,home))
    Update_Map()

    print("Block list updated:")
    for block in blockList:
        print(block)

def Update_Map():
    global mapMat
    global blockList
    global mapSize
    Init_MapMat()
    
    O = np.array([[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1],
                  [0.1,0.1,0.1,  1,  1,  1,0.1,0.1,0.1],
                  [0.1,0.1,  1,  1,  1,  1,  1,0.1,0.1],
                  [0.1,  1,  1,  1,  1,  1,  1,  1,0.1],
                  [0.1,  1,  1,  1,  1,  1,  1,  1,0.1],
                  [0.1,  1,  1,  1,  1,  1,  1,  1,0.1],
                  [0.1,0.1,  1,  1,  1,  1,  1,0.1,0.1],
                  [0.1,0.1,0.1,  1,  1,  1,0.1,0.1,0.1],
                  [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]])

    for block in blockList:
        x,y = Pos_To_Mat_Coord(block)
        x1 = max(x-4, 0)
        x2 = min(x+5, mapSize)
        y1 = max(y-4, 0)
        y2 = min(y+5, mapSize)

        p1 = 0 + max(4-x,0)
        p2 = 9 - max(x+5-mapSize,0)
        q1 = 0 + max(4-y,0)
        q2 = 9 - max(y+5-mapSize,0)

        mapMat[int(x1):int(x2), int(y1):int(y2)] += O[int(p1):int(p2),int(q1):int(q2)]

def Get_Neighbours(coord):
    neighbours = []
    if coord[0] > 0:
        neighbours.append((coord[0]-1,coord[1]))
    if coord[0] < mapSize - 1:
        neighbours.append((coord[0]+1,coord[1]))
    if coord[1] > 0:
        neighbours.append((coord[0],coord[1]-1))
    if coord[1] < mapSize - 1:
        neighbours.append((coord[0],coord[1]+1))
    return neighbours

def Reconstruct_Path(came_from, start, goal):
    global path
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional

def Find_Path():
    global mapMat
    global target
    start = Pos_To_Mat_Coord((gps.getValues()[0], gps.getValues()[2]))
    goal = Pos_To_Mat_Coord(target)

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        for next in Get_Neighbours(current):
            # Any grid containing part of an obstacle has 10000 cost to enter
            if came_from[current] != None:
                last_step = (current[0]-came_from[current][0], current[1]-came_from[current][1])
            else:
                last_step = (0, 0)
            this_step = (next[0]-current[0], next[1]-current[1])
            dot = last_step[0]*this_step[1] + last_step[1]*this_step[0]

            new_cost = cost_so_far[current] + 1 + mapMat[next[0]][next[1]] * 10000 + abs(dot) * 0.1
            if (next not in cost_so_far) or (new_cost < cost_so_far[next]):
                cost_so_far[next] = new_cost
                priority = new_cost + Steps_Between(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    Reconstruct_Path(came_from, start, goal)
    
    '''
    print(mapMat)
    print("path")
    for node in path:
        print(node)
    print("start")
    print(cost_so_far[start], ", ", start)
    print("goal")
    print(cost_so_far[goal], ", ", goal)
    '''

def Detect_Colour():
    global IsRed
    global IsBlue
    cameraData = camera.getImage()
    r = camera.imageGetRed(cameraData, camera.getWidth(),0,0)
    b = camera.imageGetBlue(cameraData, camera.getWidth(),0,0)
    if r > 70 and r-b > 20:
        IsRed = True
        IsBlue = False
    elif b > 70 and b-r > 20:
        IsBlue = True
        IsRed = False
    else:
        IsBlue = False
        IsRed = False

    '''
    print("r:", r)
    print("b:", b)
    print("IsRed:", IsRed)
    print("IsBlue:", IsBlue)
    '''

def Long_Range_Detection():
    global object_detected_counter

    if object_detected_counter[-1] == 0:
        ds_value = ds_long.getValue()
        if ds_value > 0.5:
            object_detected_counter.append(detection_counter)

            bearing = Get_Robot_Bearing()
            distance = 0.7611*pow(ds_value,-0.9313)-0.1252 + (0.12-0.078)
            x = gps.getValues()[0] - distance*np.sin(bearing*np.pi/180)
            z = gps.getValues()[2] + distance*np.cos(bearing*np.pi/180)
            #Avoid: walls, object within red and blue squares, object already recorded
            if (abs(x) < 1.11 and abs(z) < 1.11) and not (x > 0.78 and (z > 0.78 or z < -0.78)):
                IsExisting = False
                for block in blockList:
                    if abs(x - block[0]) < 0.102 and abs(z - block[1]) < 0.102:
                        IsExisting = True
                if not IsExisting:
                    Update_List((x,z),True)
                    if currentState == 1:
                        Find_Path()
    else:
        object_detected_counter.append(0)

def Float_To_ByteArray(f):
    return struct.pack('f', f)

###State Functions###

def Enter_State(state):
    global currentState
    global init_detection_counter
    global detection_counter
    global grabCounter
    global stepBackCounter
    global init_missed_counter
    global missed_counter
    global IsTurningRight
    global sendListCounter
    
    print("ENTER STATE ", state)
    if state == 0:
        detection_counter = init_detection_counter
    elif state == 1:
        Find_Path()
    elif state == 2:
        grabCounter = 10
    elif state == 3:
        print("Finished!")
        sendListCounter = len(blockList) * 2 - 1
    elif state == 4:
        if currentState == 2:
            stepBackCounter = 23
        else:
            stepBackCounter = 15
    elif state == 5:
        print("Colour not detected")
        missed_counter = init_missed_counter
        IsTurningRight = True
    
    currentState = state

# Spin 360 and scan with long-range distance sensor to record GLOBAL positions of blocks, but DO NOT record walls and other robots
def Scan():
    global target
    global detection_counter
    global IsNewScanEmpty
    
    if detection_counter > 0:
        left_speed = 2.0
        right_speed = -2.0

        Long_Range_Detection()
        detection_counter -= 1
    else:
        left_speed = 0.0
        right_speed = 0.0

        if len(toGrabList) != 0:
            target = toGrabList[0]
        else:
            IsNewScanEmpty = True
            target = home
        Enter_State(1)
        
    wheels[0].setVelocity(left_speed)
    wheels[1].setVelocity(right_speed)
    
# Follow Path
def Follow_Path():
    global path
    global curIndex
    global target
    global lastGrabPosition
    global blocksCollected
    global IsColourChecked

    pos = (gps.getValues()[0], gps.getValues()[2])
    curIndex = Find_Index_Of_Closest_Node()
    leftSpeed = 3.6
    rightSpeed = 3.6
    
    ds_value = ds_short.getValue()
    distance = 0.1594*pow(ds_value,-0.8533)-0.02916
    if distance < 0.1 and IsMovingToGrab and not IsColourChecked:
        Detect_Colour()
        if IsRed:
            print("Red. Correct Colour")
            IsColourChecked = True
        else:
            print("Wrong Colour")
            toGrabList.remove(target)
            Enter_State(4)

    if len(path) - curIndex > 3:
        nextPos = Mat_Coord_To_Pos(path[curIndex+1])
        dirAng = Angle_Between(Get_Direction(), (nextPos[0]-pos[0],nextPos[1]-pos[1]))

        leftSpeed += dirAng / 10
        rightSpeed -= dirAng / 10

        if dirAng > 1:
            leftSpeed = 1.0
            rightSpeed = -1.0
        if dirAng < -1:
            leftSpeed = -1.0
            rightSpeed = 1.0

        if dirAng > 5:
            leftSpeed = 2.0
            rightSpeed = -2.0
        if dirAng < -5:
            leftSpeed = -2.0
            rightSpeed = 2.0

        if dirAng > 10:
            leftSpeed = 3.6
            rightSpeed = -3.6
        if dirAng < -10:
            leftSpeed = -3.6
            rightSpeed = 3.6
        
        if abs(dirAng) > 1 and IsMovingToGrab and distance > 0.2:
                Long_Range_Detection()

    elif len(path) - curIndex == 3:
        if not target == lastGrabPosition and not target == home and IsMovingToGrab and distance >= 0.2:
            Enter_State(5)
    elif len(path) - curIndex < 2:
        leftSpeed = 0.0
        rightSpeed = 0.0
        if not target == lastGrabPosition:
            if not IsMovingToGrab:
                blocksCollected += 1
                print(blocksCollected, " blocks collected")
            
            if blocksCollected == 4:
                Enter_State(3)
            elif not IsNewScanEmpty:
                print("Target reached")
                Enter_State(2)
            else:
                print("Finish")
                Enter_State(3)
        else:
            print("Starting new scan")
            Enter_State(0)
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

#Toggle grabber state
def Toggle_Grabber_State():
    global grabCounter
    global IsMovingToGrab
    global target
    global home
    global IsColourChecked

    if grabCounter > 0:
        if IsMovingToGrab:
            grabber.setVelocity(2.5)
        else:
            grabber.setVelocity(-2.5)
        grabCounter -= 1
    else:
        grabber.setVelocity(0.0)
        IsMovingToGrab = not IsMovingToGrab
        IsColourChecked = False
        if not IsMovingToGrab:
            print("Block grabbed, return home")
            Update_List(target, False)
            target = home
            Enter_State(1)
        else:
            Enter_State(4)
    return

# Finish and send block data to blue bot in x,z,x,z... order
def Finish():
    global message_to_send
    global sendListCounter
    if sendListCounter < 0:
        message_to_send = b'0xff'
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
    else:
        index = sendListCounter // 2
        axis = (sendListCounter + 1) % 2
        message_to_send = Float_To_ByteArray(blockList[index][axis])
        wheels[0].setVelocity(3.6)
        wheels[1].setVelocity(3.6)
    sendListCounter -= 1

#Step back after releasing block or encountering wrong block
def Step_Back():
    global stepBackCounter
    global target
    global toGrabList
    if stepBackCounter > 0:
        wheels[0].setVelocity(-3.6)
        wheels[1].setVelocity(-3.6)
        stepBackCounter -= 1
    else:
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        if len(toGrabList) > 0:
            print("Grab next block")
            target = toGrabList[0]
            Enter_State(1)
        else:
            curCoord = Pos_To_Mat_Coord((gps.getValues()[0], gps.getValues()[2]))
            homeCoord = Pos_To_Mat_Coord(home)
            if Steps_Between(curCoord,homeCoord) < 10:
                print("Move to new scan")
                target = lastGrabPosition
                Enter_State(1)
            else:
                print("Starting new scan")
                Enter_State(0)

#Look for block if color sensor missed it
def Look_For_Missed_Block():
    global missed_counter
    global IsTurningRight

    leftSpeed = 0.0
    rightSpeed = 0.0
    if missed_counter > 0:
        if IsTurningRight:
            leftSpeed = 2.0
            rightSpeed = -2.0
        else:
            leftSpeed = -2.0
            rightSpeed = 2.0
        missed_counter -= 1
    else:
        if IsTurningRight:
            missed_counter = init_missed_counter * 2
            IsTurningRight = False
        else:
            print("Block not found")
            toGrabList.remove(target)
            Enter_State(4)

    ds_value = ds_short.getValue()
    distance = 0.1594*pow(ds_value,-0.8533)-0.02916
    if distance < 0.1:
        Detect_Colour()
        if IsRed:
            print("Red. Correct Colour")
            leftSpeed = 3.6
            rightSpeed = 3.6
            Enter_State(2)
        else:
            print("Wrong Colour")
            toGrabList.remove(target)
            Enter_State(4)

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

###Main Loop###

while robot.step(TIME_STEP) != -1:
    if currentState == 0:
        Scan()

    if currentState == 1:
        Follow_Path()

    if currentState == 2:
        Toggle_Grabber_State()

    if currentState == 3:
        Finish()

    if currentState == 4:
        Step_Back()

    if currentState == 5:
        Look_For_Missed_Block()
    
    emitter.send(message_to_send)