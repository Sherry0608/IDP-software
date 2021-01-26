"""go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

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
motor=[None]*4
motor_name=["wheel1","wheel2","wheel3", "wheel4"]
for i in range (0,4):
    motor[i] = robot.getDevice(motor_name[i])
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    left_speed = 2.0
    right_speed = 2.0
    
    motor[0].setVelocity(left_speed)
    motor[1].setVelocity(right_speed)
    motor[2].setVelocity(left_speed)
    motor[3].setVelocity(right_speed)
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #pass

# Enter here exit cleanup code.
