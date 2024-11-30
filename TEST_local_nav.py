from tdmclient import ClientAsync, aw
import time
import math

# This code is inspired from the exercises of the course given in weeks 3 and 4

# Constants at the start of the program
state = "global"                     # state of the navigation: "global" or "local"
obst = [0, 0, 0, 0, 0]               # measurements of the front proximity sensors, not taking into account the back sensors
motor_speed = [0, 0]                 # motor speed of the robot [left, right]
sensor_scale = 200   # scale of the proximity sensors
obstThrL = 10        # low obstacle threshold to switch state local->global (equivalent to 2000)
obstThrH = 20        # high obstacle threshold to switch state global->local (equivalent to 4000)
obstSpeedGain = 5    # /100 (actual gain: 5/100=0.05)
initial_speed = 100  # initial speed of the robot
is_goal_reached = False   # flag to check if the goal is reached
max_speed = 500           # maximum speed of the robot for safety

# TODO: Get these inputs dynamically
next_checkpoints = [(0, 0), (0, 0)]  # list of checkpoints to reach
current_position = (0, 0)            # current position of the robot

w_l = [40,  20, -20, -20, -40] # weights for the left motor
w_r = [-40, -20, -20,  20,  40] # weights for the right motor

DEBUG = True
def log(message):
    """Debug logging."""
    if DEBUG:
        print(message)

def is_checkpoint_reached(current_position, checkpoint):
    """
    Check if the robot has reached the checkpoint
    """
    return math.sqrt((current_position[0] - checkpoint[0])**2 + (current_position[1] - checkpoint[1])**2) < 10

@onevent
def simple_navigation():
    global prox_horizontal, state, obst, obstThrL, obstThrH, next_checkpoints, current_position, obstSpeedGain, initial_speed, motor_speed, is_goal_reached, w_l, w_r, sensor_scale

    # Get the measurements from the proximity sensors
    for i in range(len(obst)):
        obst[i] = prox_horizontal[i] // sensor_scale

    # check if the state should be switched
    if state == "local":
        if all(sensor < obstThrL for sensor in prox_horizontal):
            state = "global"
            print("Switching to global navigation: No detected obstacle anymore..")
    elif state == "global":
        if any(sensor > obstThrH for sensor in prox_horizontal):
            state = "local"
            print("Switching to local navigation: Obstacle detected..")

    # Navigation
    if state == "global":
        motor_left_target = initial_speed
        motor_right_target = initial_speed

    elif state == "local":
        y = [0, 0]
        for i in range(len(obst)):
            y[0] += w_l[i] * obst[i]
            y[1] += w_r[i] * obst[i]
        motor_left_target = max(-max_speed, min(max_speed, y[0]))
        motor_right_target = max(-max_speed, min(max_speed, y[1]))
        log("Left motor speed: {}, Right motor speed: {}".format(motor_left_target, motor_right_target))
