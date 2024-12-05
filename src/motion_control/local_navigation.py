import asyncio
import math

# This code was mainly inspired by the homeworks of the Basics of Mobile Robotics at EPFL (Sessions 3 and 4)

obstThrL = 5                # low obstacle threshold to switch state local->global
obstThrH = 8.5              # high obstacle threshold to switch state global->local
w_l = [10, 5, -5, -5, -10]  # weights for left motor
w_r = [-10, -5, -5, 5, 10]  # weights for right motor
nb_active_sensors = 0
previous_direction = 'left'

def get_navigation_state(driver, state):
    """
    Get the navigation state based on the proximity sensor values.
    Args:
        driver (Driving): Instance of the Driving class.
        state (int): Current navigation state.
                    state = 0 -> global navigation
                    state = 1 -> local navigation
                    state = 2 -> kidnapping
    Returns:
        current_state (int): New navigation state.
    """
    global nb_active_sensors
    current_state = state
    obst = asyncio.run(driver.get_prox_horizontal())
    ground = asyncio.run(driver.get_prox_ground())

    if ground[0] == 0 or ground[1] == 0:
        current_state = 2

    elif state == 1:
        if all(sensor < obstThrL for sensor in obst):
            if nb_active_sensors != 0:
                # Move by the number of active sensors + the width of the robot
                driver.move(60 + (10 * nb_active_sensors))
            
                # Smoothing step to avoid repeated state switching
                if previous_direction == 'left':
                    driver.turn(math.pi/4)
                elif previous_direction == 'right':
                    driver.turn(-math.pi/4)
                driver.move(100) 

                nb_active_sensors = 0

            current_state = 0

    elif state == 0: # global navigation
        if any(sensor > obstThrH for sensor in obst):
            nb_active_sensors = sum(sensor > 0 for sensor in obst)
            current_state = 1

    elif state == 2:
        current_state = 2
    return current_state, obst

def calculate_new_motors_speed(obst):
    """
    Calculate the new motor speeds based on the proximity sensor values.
    It also updates the previous direction of the robot.
    Args:
        obst (list): List of the first 5 proximity sensor values.
    Returns:
        y (list): List of motor speeds for the left and right motor.
    """
    global previous_direction
    y = [0, 0]
    for i in range(len(obst)):
        y[0] = y[0] + w_l[i] * obst[i] # Left motor
        y[1] = y[1] + w_r[i] * obst[i] # Right motor
    previous_direction = 'right' if y[0] < y[1] else 'left'
    return y[0], y[1]

