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

    # get robot pose
    cam.update(corners= False, obstacles_goal=False, show_all=False)
    robot_pose_px = cam.get_robot_pose()

    # kalman filter, position in pixels and angle in radians 
    # print(">>> Filtering")
    # EKF.update_time(time.time())
    # get speeds in mm/s
    # motor_values = (driver.get_speeds(), Wheel_Distance, Scaling_Factor)
    # EKF.extended_kalman([66, 0], EKF.system_state(robot_pose_px))
    # robot_pose_mm = ([EKF.Mu[0], EKF.Mu[1]], EKF.Mu[2])
    # print("Before Filter: ",robot_pose_px)
    # print("After Filter: ",robot_pose_mm)


    #reset filter 
    # EKF.Mu = [robot_pose_mm[0][0],robot_pose_mm[0][1],robot_pose_mm[1],0,0]

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
