"""go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,DistanceSensor, Camera,GPS, Compass

# create the Robot instance.
robot = Robot()
timestep = 64
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
avoid_obstacle_counter = 0
initial_detection_counter = 560
detection_counter = initial_detection_counter
return_counter = initial_detection_counter
move_forward_counter =100
final_angle = 0
object_detected_counter = [0]
object_detected_distance = [0]
object_detected_angle = []
object_detected_counter1 = []


motor=[None]*4
motor_name=["wheel1","wheel2"]
for i in range (0,2):
    motor[i] = robot.getDevice(motor_name[i])
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0)
#get and initialize distance sensor
ds=[None]*2
ds_names = ["Sharp's IR sensor GP2Y0A02YK0F", "Sharp's IR sensor GP2D120"]
for j in range (0,2):
    ds[j] = robot.getDevice(ds_names[j])
    ds[j].enable(timestep)

#get Camera and enable camera
camera = robot.getDevice("camera")
camera.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)
#define object detection function
def object_detection():
    
    while robot.step(timestep) != -1:
        
        global final_angle
        global detection_counter
        initial_angle = compass.getValues()[0]
        
       
        if detection_counter > 0:
           detection_counter = detection_counter -1
           left_speed = 1.0
           right_speed = -1.0
           motor[0].setVelocity(left_speed)
           motor[1].setVelocity(right_speed)
          
           ds_value = ds[0].getValue()
           compass_value = compass.getValues()
           if ds_value >0.5:
               if object_detected_counter[-1] == 0:
                   object_detected_counter.append(detection_counter)
                   object_detected_angle.append(compass_value[0])
                   distance = 0.7611*ds_value**(-0.9313)-0.1252
                   object_detected_distance.append(distance)
           
           elif object_detected_counter[-1] != 0:
               object_detected_counter.append(0)
            
        else:
            left_speed = 0.0
            right_speed = 0.0
            
            motor[0].setVelocity(left_speed)
            motor[1].setVelocity(right_speed)
            break
           
        
        return object_detected_counter, object_detected_distance, object_detected_angle

def return_initial():
    global initial_detection_counter
    global return_counter
    
    if detection_counter ==0 and return_counter > 0:
        return_counter  = return_counter -1
        left_speed = -1.0
        right_speed = 1.0
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
        
    elif detection_counter == 0 and return_counter ==0:
        left_speed = 0.0
        right_speed = 0.0
        
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
        
        
def object_detection_again():
        
        global detection_counter
        initial_angle = compass.getValues()[0]
        if detection_counter == 0 and return_counter ==0:
            detection_counter = initial_detection_counter 
            if detection_counter > 0:
               detection_counter = detection_counter -1
               left_speed = 1.0
               right_speed = -1.0
               motor[0].setVelocity(left_speed)
               motor[1].setVelocity(right_speed)
               
               for j in range (0,2):
                   ds_values[j] = ds[j].getValue()
               if ds_values[0] <1000.0 or ds_values[1]<1000.0:
                   object_detected.append(detection_counter)
                
            if detection_counter == 0:
               left_speed = 0.0
               right_speed = 0.0
            
               motor[0].setVelocity(left_speed)
               motor[1].setVelocity(right_speed)
              
            
        return object_detected
# Main loop:
# - perform simulation steps until Webots is stopping the controller

object_detection()
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #print(ds[1].getValue())
    left_speed = 0.0
    right_speed = 0.0
    
    ds_value = ds[0].getValue()
    cameraData = camera.getImage()
    r_value = camera.imageGetRed(cameraData, camera.getWidth(),0,0)
    b_value = camera.imageGetBlue(cameraData, camera.getWidth(),0,0)
    #print(r_value)
    
    #gps_value = gps.getValues()
    #print(gps_value)
    
    compass_value = compass.getValues()
    
    print(compass_value[0])
    
    break
   
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
    

    
    return_initial()
    if detection_counter == 0 and return_counter ==0:
        break
        
for i in range(len(object_detected_counter)):
    if object_detected_counter[i] != 0:
        object_detected_counter1.append(object_detected_counter[i])
        
object_detected_distance.remove(0)
print(object_detected_counter1)
print(object_detected_distance)
print(object_detected_angle)
detected_dict = dict(zip(object_detected_counter1, object_detected_distance))
print(detected_dict) 

detection_counter = initial_detection_counter


object_detection()




        
for i in range(len(object_detected_counter)):
    if object_detected_counter[i] != 0:
        object_detected_counter1.append(object_detected_counter[i])
               
print(object_detected_counter1)
    

while robot.step(timestep) != -1:
    if move_forward_counter >0:
        left_speed = 1.0
        right_speed = 1.0
 
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
        
        move_forward_counter = move_forward_counter -1
    
    if move_forward_counter == 0:
        left_speed = 0.0
        right_speed = 0.0
 
        motor[0].setVelocity(left_speed)
        motor[1].setVelocity(right_speed)
       
        break


    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass

# Enter here exit cleanup code.