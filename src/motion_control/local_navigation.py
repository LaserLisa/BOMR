import asyncio

obstThrL = 5          # low obstacle threshold to switch state local->global (equivalent to 2000)
obstThrH = 8.5          # high obstacle threshold to switch state global->local (equivalent to 4000)
w_l = [10, 5, -5, -5, -10]  # weights for left motor
w_r = [-10, -5, -5, 5, 10]  # weights for right motor
nb_active_sensors = 0

def get_navigation_state(driver, state):
    """
    Update the navigation state based on the proximity sensor values.
    Args:
        obst (list): List of proximity sensor values.
        driver (Driving): Instance of the Driving class.
        state (int): Current state of the navigation.
    Returns:
        current_state (int): Updated state of the navigation: "global"=0 or "local"=1.
    """
    global nb_active_sensors
    current_state = state
    obst = asyncio.run(driver.get_prox_horizontal())
    ground = asyncio.run(driver.get_prox_ground())
    # state = 0 -> global navigation
    # state = 1 -> local navigation
    # state = 2 -> kidnapping
    if ground[0] == 0 or ground[1] == 0:
        current_state = 2
    elif state == 1:
        if all(sensor < obstThrL for sensor in obst):
            if nb_active_sensors != 0:
                print("Entered loop to move")
                driver.move(50 + (10 * nb_active_sensors))
                nb_active_sensors = 0
            current_state = 0
    elif state == 0: # global navigation
        if any(sensor > obstThrH for sensor in obst):
            nb_active_sensors = sum(sensor > 0 for sensor in obst)
            print("nb of active sensors: ", nb_active_sensors)
            current_state = 1
    return current_state, obst

def calculate_new_motor_speed(obst):
    """
    Calculate the new motor speeds based on the proximity sensor values and the weights for the left and right motor.
    Args:
        obst (list): List of proximity sensor values.
        w_l (list): List of weights for the left motor.
        w_r (list): List of weights for the right motor.
    Returns:
        motor_left_target (int): New motor speed for the left motor.
        motor_right_target (int): New motor speed for the right motor.
    """
    y = [0, 0]
    for i in range(len(obst)):
        y[0] = y[0] + w_l[i] * obst[i]
        y[1] = y[1] + w_r[i] * obst[i]
    return y[0], y[1]