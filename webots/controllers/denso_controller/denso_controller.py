"""denso_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import numpy as np
from controller import Robot


def set_position(motors, q):
    for motor, q_i in zip(motors, q):
        motor.setPosition(q_i)


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motors = [robot.getDevice(f'joint{i}') for i in range(1, 7)]
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

t = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    q = np.sin(t) * np.ones(6)
    set_position(motors, q)
    t += 0.01
    pass

# Enter here exit cleanup code.
