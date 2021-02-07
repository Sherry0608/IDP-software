from controller import Robot
import numpy as np

###Set up parameters and sensors###

TIME_STEP = 64
robot = Robot()

# Distance sensors, return value can be matched to distance with a function
ds = []
dsNames = ['ds_right', 'ds_left','Sharp IR sensor GP2Y0A02YK0F']

for i in range(3):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

# Motors on each wheel 
wheels = []
wheelsNames = ['wheel1', 'wheel2']

for i in range(len(wheelsNames)):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# Compass returning cartesian vector of north direction relative to robot bearing
cp = robot.getDevice("compass")
cp.enable(TIME_STEP)

# GPS returning cartesian vector of robot global position
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

###Global variables###

# python doesn't even have constants I mean what the fuck
startPos = np.array([1,1])

# 0 for scanning, 1 for moving towards target, 2 for toggling grabber state, 3 for returning, 4 for finished
currentState = 0

# flag for grabber state
IsGrabbing = False

# flag for finishing collection
IsAllBlockCollected = False

# List of the global center position of all blocks in the playground, with collected blocks removed during runtime
blockList = []

# Represent map by a 48x48 matrix of grid size 0.05m, 1 if obstacle exists, otherwise 0
#mapMat = np.zeros(48,48)

# Global position (X,Z) of current and next target for state 1 (MoveToTarget)
curTarget = np.zeros(2)
nextTarget = np.zeros(2)

# Timestep counter after detecting obstacle
avoidObstacleCounter = 0

#Timestep counter for scanning function
initial_detection_counter = 280
detection_counter = initial_detection_counter
object_detected_counter = [0]
object_detected_distance = []
object_detected_angle = []
global_coordinate_x = []
global_coordinate_z = []
coordinate_dict = {}

###Functions###

# Spin 360
# record GLOBAL positions of blocks, but DO NOT record walls and other robots
def Scan():
    # call UpdateListAndMap(coords, True)
        
        global final_angle
        global detection_counter
        global coordinate_dict
        initial_angle = cp.getValues()[0]
        
       
        if detection_counter > 0:
           detection_counter = detection_counter -1
           left_speed = 2.0
           right_speed = -2.0
           wheels[0].setVelocity(left_speed)
           wheels[1].setVelocity(right_speed)
          
           ds_value = ds[2].getValue()
           compass_value = cp.getValues()
           if ds_value >0.5:
               if object_detected_counter[-1] == 0:
                   object_detected_counter.append(detection_counter)
                   object_detected_angle.append(GetBearingIndegrees())
                   distance = 0.7611*(ds_value**(-0.9313))-0.1252
                   object_detected_distance.append(distance)
                   x_coordinate = gps.getValues()[0] - distance*np.sin(GetBearingIndegrees()*np.pi/180)
                   z_coordinate = gps.getValues()[2] + distance*np.cos(GetBearingIndegrees()*np.pi/180)
                   #try to not record the coordinate of the walls
                   if np.abs(x_coordinate) < 1.15 and np.abs(z_coordinate) < 1.15:
                       global_coordinate_x.append(x_coordinate)
                       global_coordinate_z.append(z_coordinate)    
           
           elif object_detected_counter[-1] != 0:
               object_detected_counter.append(0)
            
        else:
            left_speed = 0.0
            right_speed = 0.0
            
            wheels[0].setVelocity(left_speed)
            wheels[1].setVelocity(right_speed)
            
            #if k < len(object_detected_distance):
             #   x_coordinate = gps.getValues()[0] - object_detected_distance[k]*np.sin(object_detected_angle[k]*np.pi/180)
             #   z_coordinate = gps.getValues()[2] + object_detected_distance[k]*np.cos(object_detected_angle[k]*np.pi/180)
             #   k = k + 1
             #  #if (round(x_coordinate,4) not in global_coordinate_x):
              #  global_coordinate_x.append(x_coordinate)
              #  global_coordinate_z.append(z_coordinate)
                
            print(object_detected_counter)
            print(object_detected_distance)
            print(object_detected_angle)
            detected_dict = dict(zip(object_detected_angle, object_detected_distance))
            coordinate_dict = dict(zip(global_coordinate_x, global_coordinate_z))
            print(detected_dict)
            print(global_coordinate_x)
            print(global_coordinate_z)
            print(coordinate_dict)
        
        
        return object_detected_counter, object_detected_distance, object_detected_angle,global_coordinate_x,global_coordinate_z,coordinate_dict


# Move to curTarget, navigating with A* on mapMat to vicinity of target, then move straight in
def MoveToTarget():
    return
    
def ToggleGrabberState():
    # call UpdateListAndMap(coords, False) if IsGrabbing == True
    return

# Return to startPos, navigating with A* on mapMat to vicinity of target, then move straight in
def Return():
    return

# Remain stationary
def Finish():
    return

# Obtain coordinate of block outline and use their mid-point to find the block center
# Consider the block a cylinder and mark every point it touches as 1 on mapMat
def UpdateListAndMap(coords, isAdd):
    return

def AvoidObstacle():
    global avoidObstacleCounter
    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 100
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

def TestCompass():
    print("Compass")
    print("X:", cp.getValues()[0])
    print("Y:", cp.getValues()[1])
    print("Z:", cp.getValues()[2])
    print("Bearing:", GetBearingIndegrees())
    print("#####################")

def TestGPS():
    print("GPS")
    print("X:", gps.getValues()[0])
    print("Y:", gps.getValues()[1])
    print("Z:", gps.getValues()[2])
    print("#####################")

def GetBearingIndegrees():
    radian = np.arctan2(cp.getValues()[0], cp.getValues()[2])
    bearing = (radian - 1.5708) / np.pi * 180.0
    if (bearing < 0.0):
        bearing = bearing + 360.0
    return bearing

###Main Loop###

while robot.step(TIME_STEP) != -1:
    if currentState == 0:
        Scan()
        # Transition to next state after 360 scan is finished
        # If blockList[] is not empty, transition to state 1 (MoveToTarget)
        # Else, flag IsAllBlockCollected = True and transition to state 3 (Return)

    if currentState == 1:
        MoveToTarget()
        # Transition to next state after curTarget is reached
        # If blockList[] is not empty, transition to state 2 (ToggleGrabberState) for grabbing
        # Else, transition to state 0 (Scan)

    if currentState == 2:
        ToggleGrabberState()
        # Transition to next state after grabber motion is finished
        # When Grabbing:
        #   Updates blockList[]
        #   Always transition to state 3 (Return)
        # When Releasing:
        #   If IsAllBlockCollected = True, transition to state 4 (Finished)
        #   Elif blockList[] is not empty, transition to state 1 (MoveToTarget) with nextTarget being the next item in blockList[], offsetted to give space for grabbing
        #   Else, transition to state 1 (MoveToTarget) with nextTarget being curTarget

    if currentState == 3:
        Return()
        # Transition to next state after home is reached
        # Always transition to state 2 (ToggleGrabberState) for releasing

    
    if currentState == 4:
        Finish()
        # Remain stationary
    
    ds_value = ds[2].getValue()

    Scan()
    
    

    
    #AvoidObstacle()
    TestGPS()
    #TestCompass()