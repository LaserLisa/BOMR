import cv2
import numpy as np
from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from driving import Driving
import time
import threading
import sys

running = False
DEBUG = False

def update_camera_and_kalman(cam: camera.Camera):
    global running
    # global driver
    print("Starting update_camera_and_kalman thread")
    while running:
        # Update the camera and perform filtering
        cam.update(corners=False, obstacles_goal=False, show_all=False)
        robot_pose_px = cam.get_robot_pose()

        # ekf.dt = driver.get_time()
        # ekf.extended_kalman(
        #     ekf.u_input(driver.get_l_speeds(), driver.get_r_speeds(), Wheel_Distance, Scaling_Factor),
        #     ekf.system_state(robot_pose_px),
        # )
        # robot_pose_mm = ([ekf.Mu[0], ekf.Mu[1]], ekf.Mu[2])

        # # Reset filter
        # ekf.Mu = [robot_pose_mm[0][0], robot_pose_mm[0][1], robot_pose_mm[1], 0, 0]

        # Display the frame and map
        cam.display_map()
        if DEBUG:
            frame = cam.get_current_frame()
            cv2.imshow("Camera", frame)
        cv2.waitKey(1)

        # time.sleep(0.01)  # Prevent excessive CPU usage

    print("update_camera_and_kalman thread exiting")



def motion_control(driver: Driving, robot_pose: tuple):
    global running
    print("Starting motion_control thread")
    
    (driver.x, driver.y, driver.dir) = (robot_pose[0][0], robot_pose[0][1], robot_pose[1])
    for i in range(len(checkpoints)):
        driver.move_to_checkpoint(driver.x, driver.y, driver.dir, checkpoints[i][0], checkpoints[i][1])
        
    print("ITERATION COMPLETED ----------------------------------------------")
    print("motion_control thread exiting")


if __name__ == "__main__":
    running = True
    print("Initializing camera...")
    cam = camera.Camera(0, window_size=2)
    pix2mm = cam.pixel2mm

    print("Initializing map...")
    cam.initialize_map(show=True, show_all=False)
    
    robot_pose_px = cam.get_robot_pose()
    map = cam.get_map()
    goal = cam.get_goal_position()
    print("Getting checkpoints...")
    checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal, pix2mm)[1:]
    print(len(checkpoints))
    #checkpoints = [goal]
    cam.set_checkpoints(checkpoints)

    for val in checkpoints:
        print(val, "\n")

    print("Initializing Thymio...")
    driver = Driving()

    # print(">> Initializing filter")
    # ekf = Extended_Kalman_Filter()
    # ekf.Sigma = np.eye(5)
    # ekf.Mu = [robot_pose_px[0][0], robot_pose_px[0][1], robot_pose_px[1], 0, 0]
    # ekf.old_time = time.time()
    # Wheel_Distance = 100
    # Scaling_Factor = 3

    # Start threads
    t1 = threading.Thread(target=update_camera_and_kalman, args=(cam, ), daemon=True)
    t2 = threading.Thread(target=motion_control, args=(driver, robot_pose_px), daemon=True)


    
    print("threads defined")
    t1.start()
    t2.start()

    # wait until threads terminate
    t1.join()
    t2.join()

    print("Releasing all objects")
    cam.release()
    print("Camera released")
    cv2.destroyAllWindows()
    print("OpenCV windows destroyed")
    driver.__del__()
    print("Driver released")
    
    print("Active threads after join:")
    print(threading.enumerate())

    print("all threads have completed")