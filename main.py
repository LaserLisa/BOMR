import cv2
import numpy as np
import time
import threading

from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from src.motion_control.driving import Driving

running = False
DEBUG = False

def init() -> tuple[camera.Camera, Driving, Extended_Kalman_Filter]:
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
    print(checkpoints)
    #checkpoints = [goal]
    cam.set_checkpoints(checkpoints)

    for val in checkpoints:
        print(val, "\n")

    print("Initializing Thymio...")
    driver = Driving()

    print(">> Initializing filter")
    ekf = Extended_Kalman_Filter(pix2mm)
    ekf.Mu = [robot_pose_px[0][0], robot_pose_px[0][1], robot_pose_px[1], 0, 0]

    return cam, driver, ekf, checkpoints

def update_camera_and_kalman(cam: camera.Camera):
    global running
    # global driver
    print("Starting update_camera_and_kalman thread")
    while running:
        # Update the camera and perform filtering
        cam.update(corners=False, obstacles_goal=False, show_all=False)
        robot_pose_px = cam.get_robot_pose()

        l_speed, r_speed, dt = driver.get_l_speeds(), driver.get_r_speeds() , driver.get_time()
        print(f"robot speed kalman: {l_speed}\t {r_speed}")
        ekf.Kalman_main(l_speed, r_speed, dt, robot_pose_px)
        robot_pose_mm = ([ekf.Mu[0], ekf.Mu[1]], ekf.Mu[2])

       
        # Display the frame and map
        cam.display_map()
        if DEBUG:
            frame = cam.get_current_frame()
            cv2.imshow("Camera", frame)
        cv2.waitKey(1)

        # time.sleep(0.01)  # Prevent excessive CPU usage

    print("update_camera_and_kalman thread exiting")



def motion_control(driver: Driving, camera: camera.Camera, checkpoints: list):
    global running
    print("Starting motion_control thread")
    robot_pose = camera.get_robot_pose()
    (driver.x, driver.y, driver.dir) = (robot_pose[0][0], robot_pose[0][1], robot_pose[1])
    for i in range(len(checkpoints)):
        driver.move_to_checkpoint(driver.x, driver.y, driver.dir, checkpoints[i][0], checkpoints[i][1])
    
    running = False
    print("ITERATION COMPLETED ----------------------------------------------")
    print("motion_control thread exiting")


if __name__ == "__main__":
    running = True
    cam, driver, ekf, checkpoints = init()

    # Start threads
    t1 = threading.Thread(target=update_camera_and_kalman, args=(cam, ), daemon=True)
    t2 = threading.Thread(target=motion_control, args=(driver, cam, checkpoints), daemon=True)


    
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
