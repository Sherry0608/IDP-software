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

###Global variables###

robotWidth = 0.16
robotLength = 0.2
blockSize = 0.05
floorSize = 2.4
mapSize = round(floorSize / blockSize)

# Represent map by a 48x48 matrix of grid size 0.05m, 1 if obstacle exists, otherwise 0
def Init_MapMat():
    global mapMat
    mapMat = np.pad(np.zeros((mapSize - 6,mapSize - 6)), 3, 'constant', constant_values=1)
    return mapMat

# Padded with ones to represent wall
mapMat = Init_MapMat()

# List of the center global position (X,Z) of all blocks in the playground, with collected blocks removed during runtime
blockList = []

# Global position (X,Z) of home
home = (0.9,0.9)
# Global position (X,Z) of current target
target = (0,0)
# Path to current target
path = []
curIndex = 0
# 0 for scanning, 1 for moving towards target, 2 for toggling grabber state, 3 for finished
currentState = 0
# Time step counter for grabbing
grabCounter = 0
# Flags
IsGrabbing = True

#Timestep counter for scanning function
initial_detection_counter = 140
detection_counter = initial_detection_counter
object_detected_counter = [0]
object_detected_distance = []
object_detected_angle = []
global_coordinate_x = []
global_coordinate_z = []
coordinate_dict = {}

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
    if isAdd:
        blockList.append(pos)
    else:
        blockList.remove(pos)
    Update_Map()

    print("block list")
    for block in blockList:
        print(block)

def Update_Map():
    global mapMat
    Init_MapMat()
    for block in blockList:
        coord = Pos_To_Mat_Coord(block)
        mapMat[max(coord[0]-1,0)][max(coord[1]-2,0)] = 1
        mapMat[coord[0]][max(coord[1]-2,0)] = 1
        mapMat[min(coord[0]+1,mapSize - 1)][max(coord[1]-2,0)] = 1
        
        mapMat[max(coord[0]-2,0)][max(coord[1]-1,0)] = 1
        mapMat[max(coord[0]-1,0)][max(coord[1]-1,0)] = 1
        mapMat[coord[0]][max(coord[1]-1,0)] = 1
        mapMat[min(coord[0]+1,mapSize - 1)][max(coord[1]-1,0)] = 1
        mapMat[min(coord[0]+2,mapSize - 1)][max(coord[1]-1,0)] = 1

        mapMat[max(coord[0]-2,0)][coord[1]] = 1
        mapMat[max(coord[0]-1,0)][coord[1]] = 1
        mapMat[coord[0]][coord[1]] = 1
        mapMat[min(coord[0]+1,mapSize - 1)][coord[1]] = 1
        mapMat[min(coord[0]+2,mapSize - 1)][coord[1]] = 1

        mapMat[max(coord[0]-2,0)][min(coord[1]+1,mapSize - 1)] = 1
        mapMat[max(coord[0]-1,0)][min(coord[1]+1,mapSize - 1)] = 1
        mapMat[coord[0]][min(coord[1]+1,mapSize - 1)] = 1
        mapMat[min(coord[0]+1,mapSize - 1)][min(coord[1]+1,mapSize - 1)] = 1
        mapMat[min(coord[0]+2,mapSize - 1)][min(coord[1]+1,mapSize - 1)] = 1

        mapMat[max(coord[0]-1,0)][min(coord[1]+2,mapSize - 1)] = 1
        mapMat[coord[0]][min(coord[1]+2,mapSize - 1)] = 1
        mapMat[min(coord[0]+1,mapSize - 1)][min(coord[1]+2,mapSize - 1)] = 1

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

def Heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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

        if current == goal:
            break
        
        for next in Get_Neighbours(current):
            # Any grid containing part of an obstacle has 1000 cost to enter
            new_cost = cost_so_far[current] + 1 + mapMat[next[0]][next[1]] * 1000
            if (next not in cost_so_far) or (new_cost < cost_so_far[next]):
                #if (next in cost_so_far) and new_cost < cost_so_far[next]:
                #    print(new_cost, ", ", next)
                cost_so_far[next] = new_cost
                priority = new_cost + Heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    Reconstruct_Path(came_from, start, goal)

    print(mapMat)
    print("path")
    for node in path:
        print(node)
    print("start")
    print(cost_so_far[start], ", ", start)
    print("goal")
    print(cost_so_far[goal], ", ", goal)

###State Functions###

def Enter_State(state):
    global currentState
    global initial_detection_counter
    global grabCounter
    
    print("ENTER STATE ", state)
    if state == 0:
        initial_detection_counter = 280
    elif state == 1:
        Find_Path()
    elif state == 2:
        grabCounter = 25
    elif state == 3:
        print("Finished!")
    
    currentState = state


# Spin 360 and scan with long-range distance sensor to record GLOBAL positions of blocks, but DO NOT record walls and other robots
def Scan():
    global target
    target = blockList[0]

    global detection_counter
    global coordinate_dict
    
    if detection_counter > 0:
        detection_counter = detection_counter -1
        left_speed = 2.0
        right_speed = -2.0
        
        ds_value = ds_long.getValue()
        if ds_value >0.5:
            if object_detected_counter[-1] == 0:
                object_detected_counter.append(detection_counter)
                object_detected_angle.append(Get_Robot_Bearing())
                distance = 0.7611*(ds_value**(-0.9313))-0.1252
                object_detected_distance.append(distance)
                 ##0.06 is for correcting the x coordinate. It has been consistently off by around 0.06 after testing probably because I am not taking the middle value
                if Get_Robot_Bearing() < 180:
                    x_coordinate = gps.getValues()[0] - ((distance*np.sin(Get_Robot_Bearing()*np.pi/180))+ 0.08)
                    z_coordinate = gps.getValues()[2] + distance*np.cos(Get_Robot_Bearing()*np.pi/180)
                if Get_Robot_Bearing() >= 180:
                    x_coordinate = gps.getValues()[0] - ((distance*np.sin(Get_Robot_Bearing()*np.pi/180))- 0.08)
                    z_coordinate = gps.getValues()[2] + distance*np.cos(Get_Robot_Bearing()*np.pi/180)
                #try to not record the coordinate of the walls
                if np.abs(x_coordinate) < 1.15 and np.abs(z_coordinate) < 1.15:
                    global_coordinate_x.append(x_coordinate)
                    global_coordinate_z.append(z_coordinate)    
        
        elif object_detected_counter[-1] != 0:
            object_detected_counter.append(0)
        
    else:
        left_speed = 0.0
        right_speed = 0.0

        print(object_detected_counter)
        print(object_detected_distance)
        print(object_detected_angle)
        detected_dict = dict(zip(object_detected_angle, object_detected_distance))
        coordinate_dict = dict(zip(global_coordinate_x, global_coordinate_z))
        print(detected_dict)
        print(global_coordinate_x)
        print(global_coordinate_z)
        print(coordinate_dict)
        Enter_State(1)
        
    wheels[0].setVelocity(left_speed)
    wheels[1].setVelocity(right_speed)
    # return object_detected_counter, object_detected_distance, object_detected_angle,global_coordinate_x,global_coordinate_z,coordinate_dict
    
# Follow Path
def Follow_Path():
    global path
    global curIndex
    global target

    pos = (gps.getValues()[0], gps.getValues()[2])
    curIndex = Find_Index_Of_Closest_Node()
    leftSpeed = 1.0
    rightSpeed = 1.0
    
    if curIndex+2 < len(path):
        nextPos = Mat_Coord_To_Pos(path[curIndex+1])
        dirAng = Angle_Between(Get_Direction(), (nextPos[0]-pos[0],nextPos[1]-pos[1]))
    
        if dirAng > 1:
            leftSpeed = 1.0
            rightSpeed = -1.0
        if dirAng < -1:
            leftSpeed = -1.0
            rightSpeed = 1.0
    else:
        leftSpeed = 0.0
        rightSpeed = 0.0
        Enter_State(2)
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

#Toggle grabber state
def Toggle_Grabber_State():
    global grabCounter
    global IsGrabbing
    global target
    global home

    if grabCounter > 0:
        if IsGrabbing:
            grabber.setVelocity(1.0)
        else:
            grabber.setVelocity(-1.0)
        grabCounter -= 1
    else:
        grabber.setVelocity(0.0)
        IsGrabbing = not IsGrabbing
        if not IsGrabbing:
            print("return home")
            Update_List(target, False)
            target = home
            Enter_State(1)
        elif len(blockList) > 0:
            print("grab next block")
            target = blockList[0]
            Enter_State(1)
        else:
            print("finish")
            Enter_State(3)
    return

# Finish and remain stationary
def Finish():
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)

###Main Loop###

Update_List((0.37,0.95),True)
Update_List((0,-0.44),True)

while robot.step(TIME_STEP) != -1:
    if currentState == 0:
        Scan()

    if currentState == 1:
        Follow_Path()

    if currentState == 2:
        Toggle_Grabber_State()

    if currentState == 3:
        Finish()