"""go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,DistanceSensor, Camera,GPS, Compass

# create the Robot instance.
robot = Robot()
timestep = 300000
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
avoid_obstacle_counter = 0
initial_detection_counter = 115
detection_counter = initial_detection_counter
return_counter = initial_detection_counter
final_angle = 0
object_detected =[]

motor=[None]*4
motor_name=["wheel1","wheel2","wheel3", "wheel4"]
for i in range (0,4):
    motor[i] = robot.getDevice(motor_name[i])
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0)
#get and initialize distance sensor
ds=[None]*2
ds_names = ["ds_left","ds_right"]
for j in range (0,2):
    ds[j] = robot.getDevice(ds_names[j])
    ds[j].enable(timestep)

#get Camera and enable camera
camera = robot.getDevice("camera")
camera.enable(timestep)

gps = robot.getDevice("global_gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)
#define object detection function
def object_detection():
        
        global final_angle
        global detection_counter
        initial_angle = compass.getValues()[0]
        
       
        if detection_counter > 0:
           detection_counter = detection_counter -1
           left_speed = 1.0
           right_speed = -1.0
           motor[0].setVelocity(left_speed)
           motor[1].setVelocity(right_speed)
           motor[2].setVelocity(left_speed)
           motor[3].setVelocity(right_speed)
           for j in range (0,2):
               ds_values[j] = ds[j].getValue()
           if ds_values[0] <1000.0 or ds_values[1]<1000.0:
               object_detected.append(detection_counter)
            
        else:
           left_speed = 0.0
           right_speed = 0.0
            
           motor[0].setVelocity(left_speed)
           motor[1].setVelocity(right_speed)
           motor[2].setVelocity(left_speed)
           motor[3].setVelocity(right_speed)
            
        return object_detected

def return_initial():
    global initial_detection_counter
    global return_counter
    
    if detection_counter ==0 and return_counter > 0:
        return_counter  = return_counter -1
        left_speed = -1.0
        right_speed = 1.0
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
        motor[2].setVelocity(left_speed)
        motor[3].setVelocity(right_speed)
    elif detection_counter == 0 and return_counter ==0:
        left_speed = 0.0
        right_speed = 0.0
        
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
        motor[2].setVelocity(left_speed)
        motor[3].setVelocity(right_speed)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #print(ds[1].getValue())
    left_speed = 0.0
    right_speed = 0.0
    
    ds_values = [None]*2
    cameraData = camera.getImage()
    r_value = camera.imageGetRed(cameraData, camera.getWidth(),0,0)
    b_value = camera.imageGetBlue(cameraData, camera.getWidth(),0,0)
    #print(r_value)
    
    #gps_value = gps.getValues()
    #print(gps_value)
    
    compass_value = compass.getValues()
    
    print(compass_value[0])
    
    
   
    #if avoid_obstacle_counter > 0:
    #    avoid_obstacle_counter = avoid_obstacle_counter-1
        
    #    left_speed = 1.0
    #    right_speed = -1.0
    #else:
    #    for i in range (0,2):
    #        ds_values[i] = ds[i].getValue()
    #    if r_value > b_value:
     #       avoid_object_counter = 0
     #   elif ds_values[0] <1000.0 or ds_values[1]<1000.0:
     #       avoid_obstacle_counter = 100
    object_detection()
    print(object_detected)
    print(ds_values[0])
    return_initial()

    
   
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass

# Enter here exit cleanup code.
