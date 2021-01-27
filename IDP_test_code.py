"""go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,DistanceSensor

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
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    #print(ds[1].getValue())
    left_speed = 2.0
    right_speed = 2.0
    
    ds_values = [None]*2
    
    
    
    if avoid_obstacle_counter > 0:
        avoid_obstacle_counter = avoid_obstacle_counter-1
        
        left_speed = 1.0
        right_speed = -1.0
    else:
        for i in range (0,2):
            ds_values[i] = ds[i].getValue()
        if ds_values[0] <1000.0 or ds_values[1]<1000.0:
            avoid_obstacle_counter = 100
      
    motor[0].setVelocity(left_speed)
    motor[1].setVelocity(right_speed)
    motor[2].setVelocity(left_speed)
    motor[3].setVelocity(right_speed)
    
     
   
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass

# Enter here exit cleanup code.
