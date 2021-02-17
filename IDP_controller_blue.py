from controller import Robot
import numpy as np
from queue import *
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
emitter = robot.getDevice("emitter_blue")
emitter.setChannel(1)
emitter.setRange(-1)

#Receiver used to send data
receiver = robot.getDevice("receiver_blue")
receiver.enable(TIME_STEP)
receiver.setChannel(1)

###Global variables###

robotWidth = 0.16
robotLength = 0.2
blockSize = 0.05
floorSize = 2.4
mapSize = round(floorSize / blockSize)
message = 2

# Represent map by a 48x48 matrix of grid size 0.05m, 1 if obstacle exists, otherwise 0, padded with ones to represent wall
def Init_MapMat():
    global mapMat
    mapMat = np.pad(np.zeros((mapSize - 6,mapSize - 6)), 3, 'constant', constant_values=1)
    return mapMat

mapMat = Init_MapMat()

# List of the center global position (X,Z) of all blocks in the playground, with collected blocks removed during runtime
blockList = []
toGrabList = []
# Global position (X,Z) of home
home = (0.95,0.95)
# Global position (X,Z) of current target
target = (0,0)

path = []
curIndex = 0
currentState = 0
grabCounter = 10
IsMovingToGrab = True

#Timestep counter for scanning function
detection_counter = 71
object_detected_counter = [0]

stepBackCounter = 10

#Boolean variable IsRed and IsBlue
IsRed = False
IsBlue = False

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
    if isAdd:
        blockList.append(pos)
        toGrabList.append(pos)
    else:
        blockList.remove(pos)
        toGrabList.remove(pos)
    blockList.sort(key=lambda tup: Steps_Between(tup,home))
    toGrabList.sort(key=lambda tup: Steps_Between(tup,home))
    Update_Map()

    print("Block list updated:")
    for block in blockList:
        print(block)

def Update_Map():
    global mapMat
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
        x -= 4
        y -= 4
        mapMat[x:(int)(x+O.shape[0]), y:(int)(y+O.shape[1])] += O

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
            new_cost = cost_so_far[current] + 1 + mapMat[next[0]][next[1]] * 10000
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
    r_value = camera.imageGetRed(cameraData, camera.getWidth(),0,0)
    b_value = camera.imageGetBlue(cameraData, camera.getWidth(),0,0)
    if r_value > 70 and b_value < 30:
        IsRed = True
        IsBlue = False
    if r_value < 30 and b_value > 70:
        IsBlue = True
        IsRed = False
    if r_value > 40 and b_value > 80:
        IsBlue = False
        IsRed = False
    if r_value < 20 and b_value < 40:
        IsBlue = False
        IsRed = False
    print("#####################")
    print("r:", r_value)
    print("b:", b_value)
    print("IsRed:", IsRed)
    print("IsBlue:", IsBlue)
    print("#####################")

###State Functions###

def Enter_State(state):
    global currentState
    global detection_counter
    global grabCounter
    global stepBackCounter
    
    print("ENTER STATE ", state)
    if state == 0:
        detection_counter = 71
    elif state == 1:
        Find_Path()
    elif state == 2:
        grabCounter = 10
    elif state == 3:
        print("Finished!")
    elif state == 4:
        stepBackCounter = 10
    
    currentState = state

# Spin 360 and scan with long-range distance sensor to record GLOBAL positions of blocks, but DO NOT record walls and other robots
def Scan():
    global target
    global detection_counter
    
    if detection_counter > 0:
        detection_counter -= 1
        left_speed = 4.0
        right_speed = -4.0
        
        ds_value = ds_long.getValue()
        if ds_value > 0.5:
            if object_detected_counter[-1] == 0:
                bearing = Get_Robot_Bearing()
                object_detected_counter.append(detection_counter)

                distance = 0.7611*(ds_value**(-0.9313))-0.1252+(0.12-0.078)

                x = gps.getValues()[0] - distance*np.sin(bearing*np.pi/180)
                z = gps.getValues()[2] + distance*np.cos(bearing*np.pi/180)
                #try to not record the coordinate of the walls
                if np.abs(x) < 1.15 and np.abs(z) < 1.15:
                    Update_List((x,z),True)    
        
        elif object_detected_counter[-1] != 0:
            object_detected_counter.append(0)
        
    else:
        left_speed = 0.0
        right_speed = 0.0
        target = toGrabList[0]
        Enter_State(1)
        
    wheels[0].setVelocity(left_speed)
    wheels[1].setVelocity(right_speed)
    
# Follow Path
def Follow_Path():
    global path
    global curIndex
    global target

    pos = (gps.getValues()[0], gps.getValues()[2])
    curIndex = Find_Index_Of_Closest_Node()
    leftSpeed = 3.0
    rightSpeed = 3.0
    
    ds_value = ds_short.getValue()
    distance = 0.1594*pow(ds_value,-0.8533)-0.02916
    if (distance < 0.1 and IsMovingToGrab):
        Detect_Colour()
        if (not IsBlue):
            toGrabList.remove(target)
            Enter_State(4)

    if curIndex+2 < len(path):
        nextPos = Mat_Coord_To_Pos(path[curIndex+1])
        dirAng = Angle_Between(Get_Direction(), (nextPos[0]-pos[0],nextPos[1]-pos[1]))
    
        if dirAng > 1:
            leftSpeed = 1.0
            rightSpeed = -1.0
        if dirAng < -1:
            leftSpeed = -1.0
            rightSpeed = 1.0

        if dirAng > 45:
            leftSpeed = 4.0
            rightSpeed = -4.0
        if dirAng < -45:
            leftSpeed = -4.0
            rightSpeed = 4.0
    else:
        print("Target reached")
        Detect_Colour()
        leftSpeed = 0.0
        rightSpeed = 0.0
        Enter_State(2)
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

#Toggle grabber state
def Toggle_Grabber_State():
    global grabCounter
    global IsMovingToGrab
    global target
    global home

    if grabCounter > 0:
        if IsMovingToGrab:
            grabber.setVelocity(2.5)
        else:
            grabber.setVelocity(-2.5)
        grabCounter -= 1
    else:
        grabber.setVelocity(0.0)
        IsMovingToGrab = not IsMovingToGrab
        if not IsMovingToGrab:
            print("Block grabbed, return home")
            Update_List(target, False)
            target = home
            Enter_State(1)
        elif len(toGrabList) > 0:
            print("Block released, grab next block")
            Enter_State(4)
        else:
            print("Finish")
            Enter_State(3)
    return

# Finish and remain stationary
def Finish():
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)

#Step back after releasing block
def Step_Back():
    global stepBackCounter
    global target
    global toGrabList
    if stepBackCounter > 0:
        wheels[0].setVelocity(-8.0)
        wheels[1].setVelocity(-8.0)
        stepBackCounter -= 1
    else:
        wheels[0].setVelocity(0.0)
        wheels[1].setVelocity(0.0)
        target = toGrabList[0]
        Enter_State(1)


###Main Loop###

while robot.step(TIME_STEP) != -1:
    global message
    
    if receiver.getQueueLength() > 0:   
        message = receiver.getData()
        receiver.nextPacket()

    if currentState == 0 and message == "1":
        Scan()

    if currentState == 1:
        Follow_Path()

    if currentState == 2:
        Toggle_Grabber_State()

    if currentState == 3:
        Finish()

    if currentState == 4:
        Step_Back()