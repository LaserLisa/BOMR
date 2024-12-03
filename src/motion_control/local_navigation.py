import asyncio

obstThrL = 10          # low obstacle threshold to switch state local->global (equivalent to 2000)
obstThrH = 20          # high obstacle threshold to switch state global->local (equivalent to 4000)
w_l = [6, 5, -5, -5, -6]  # weights for left motor
w_r = [-6, -5, -5, 5, 6]  # weights for right motor

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
    current_state = state
    obst = asyncio.run(driver.get_prox_horizontal())
    ground = asyncio.run(driver.get_prox_ground())
    # state = 0 -> global navigation
    # state = 1 -> local navigation
    # state = 2 -> kidnapping
    if ground[0] == 0 or ground[1] == 0:
        current_state = 2
    elif state == 1:
        if obst[0] < obstThrL and obst[1] < obstThrL and obst[2] < obstThrL and obst[3] < obstThrL and obst[4] < obstThrL:
            driver.move(40)
            current_state = 0
    elif state == 0:
        if obst[0] > obstThrH or obst[1] > obstThrH or obst[2] > obstThrH or obst[3] > obstThrH or obst[4] > obstThrH:
            current_state = 1
    elif state == 2:
        current_state = 2
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
    print("Left motor speed: ", y[0])
    print("Right motor speed: ", y[1])
    return y[0], y[1]