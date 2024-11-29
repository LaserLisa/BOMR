import cv2
import numpy as np
from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from driving import Driving
import time
import threading


def update_camera_and_kalman(cam: camera.Camera, ekf: Extended_Kalman_Filter):
    global running
    global driver
    while running:
        # get robot pose
        cam.update(corners= False, obstacles_goal=False, show_all=False)
        robot_pose_px = cam.get_robot_pose()

        # kalman filter, position in pixels and angle in radians 
        # print(">>> Filtering")
        # EKF.update_time(time.time())
        ekf.dt = driver.get_time()
        # get speeds in mm/s
        ekf.extended_kalman(ekf.u_input(driver.get_l_speeds(), driver.get_r_speeds(), 
                                        Wheel_Distance, Scaling_Factor), ekf.system_state(robot_pose_px))
        robot_pose_mm = ([ekf.Mu[0], ekf.Mu[1]], ekf.Mu[2])
        # print("Before Filter: ",robot_pose_px)
        # print("After Filter: ",robot_pose_mm)


        #reset filter 
        ekf.Mu = [robot_pose_mm[0][0],robot_pose_mm[0][1],robot_pose_mm[1],0,0]

        # to display the current frame and the map
        frame = cam.get_current_frame()
        cam.display_map()
        cv2.imshow('Camera', frame)
        time.sleep(0.01)
        cv2.waitKey(1)

def motion_control():
    global running
    command = input("Enter command ('turn degrees' or 'move duration') or 'exit' to quit: ").strip()
    if command.lower() == "exit":
        print("Exiting program.")
        running = False


if __name__ == "__main__":
    running = True
    # Open the default camera
    print("Initalizing camera...")

    #change parameter to select camera source if multiple connected
    cam = camera.Camera(0, window_size=2) 
    pix2mm = cam.pixel2mm

    print("Intializing map...")
    cam.initialize_map(show=True)
    robot_pose_px = cam.get_robot_pose()
    map = cam.get_map()
    goal = cam.get_goal_position()
    print("gettng checkpoints")
    checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal)


    print("Initalize Thymio...")
    driver = Driving()


    # TODO: initialize kalman filter
    print(">> Initializing filter") 
    ekf = Extended_Kalman_Filter()
    ekf.Sigma = np.eye(5) # confidence of EKF
    ekf.Mu = [robot_pose_px[0][0],robot_pose_px[0][1],robot_pose_px[1],0,0] #initial values for position and orientation
    ekf.old_time = time.time()
    Wheel_Distance = 100
    Scaling_Factor = 3

    # lock = threading.Lock()
    t1 = threading.Thread(target=update_camera_and_kalman, args=(cam, ekf))
    t2 = threading.Thread(target=motion_control)

    t1.start()
    t2.start()

    t1.join()
    t2.join()
    print(">>>> Releasing all objects")
    cam.release()
    cv2.destroyAllWindows()
