import cv2
import numpy as np
from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from driving import Driving
import time

# Open the default camera
print("Initalizing camera...")

cam = camera.Camera(0, window_size=2) #change parameter to select camera source if multiple connected
pix2mm = cam.pixel2mm

print("Intializing map...")
cam.initialize_map(show=True)
robot_pose_px = cam.get_robot_pose()
map = cam.get_map()
goal = cam.get_goal_position()
print("gettng checkpoints")
checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal)


# TODO: Thymio initalization
print("Initalize Thymio...")
driver = Driving()


# TODO: initialize kalman filter
print(">> Initializing filter") 
EKF = Extended_Kalman_Filter()
EKF.Sigma = np.eye(5) # confidence of EKF
EKF.Mu = [robot_pose_px[0][0],robot_pose_px[0][1],robot_pose_px[1],0,0] #initial values for position and orientation
EKF.old_time = time.time()
Wheel_Distance = 100
Scaling_Factor = 3


while True:
    # if obstacle_detected():
    #    ...
    # else:
    # move towards next checkpoint
    # Get user input for the command
    command = input("Enter command 5 coordinates: (pos_x, pos_y, pos_angle, check_x, check_y) or 'exit' to quit: ").strip()
    if command.lower() == "exit":
        print("Exiting program.")
        break

    # Parse the command
    try:
        pos_x, pos_y, pos_angle, check_x, check_y = command.split()
        pos_x = int(pos_x)
        pos_y = int(pos_y)
        pos_angle = int(pos_angle)
        check_x = int(check_x)
        check_y = int(check_y)
        driver.move_to_checkpoint(pos_x, pos_y, pos_angle, check_x, check_y)
        '''
        if command.startswith("turn"):
            _, degrees = command.split()
            degrees = int(degrees)
            driver.turn(degrees)
        elif command.startswith("move"):
            _, duration = command.split()
            duration = int(duration)
            driver.move(duration)
        else:
            print("Invalid command. Use 'turn degrees' or 'move duration'.")
        '''
    except ValueError:
        print("Invalid input. Please ensure the correct format for 'turn degrees' or 'move duration'.")

    # get robot pose
    cam.update(corners= False, obstacles_goal=False, show_all=False)
    robot_pose_px = cam.get_robot_pose()

    # kalman filter, position in pixels and angle in radians 
    print(">>> Filtering")
    EKF.update_dt(time.time())
    # get speeds in mm/s
    motor_values = (driving.get_motor_speeds(), Wheel_Distance, Scaling_Factor)
    EKF.extended_kalman(EKF.u_input(motor_values),EKF.system_state(robot_pose_px))
    robot_pose_mm = ([EKF.Mu[0], EKF.Mu[1]], EKF.Mu[2])


    #reset filter 
    EKF.Mu = [robot_pose_mm[0][0],robot_pose_mm[0][1],robot_pose_mm[1],0,0]

    # to display the current frame and the map
    cam.update(corners= False, obstacles_goal=False, show_all=False)
    frame = cam.get_current_frame()
    cam.display_map()
    cv2.imshow('Camera', frame)


    # Press 'q' to exit the loop
    # if cv2.waitKey(1) == ord('q'):
    #    break
# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()
