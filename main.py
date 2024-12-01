import cv2
import numpy as np
import time
import threading

from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from src.motion_control.driving import Driving
from src.motion_control.helpers import checkpoint_reached
# from local_nav import obstacle_detected
def obstacle_detected():
    return False

running = False # global variable to enable disable threads
DEBUG = False # global variable to enable/disable debug messages

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
    # checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal, pix2mm)[1:]
    # print(checkpoints)
    checkpoints = [goal]
    cam.set_checkpoints(checkpoints)

    for val in checkpoints:
        print(val, "\n")

    print("Initializing Thymio...")
    driver = Driving()

    print(">> Initializing filter")
    ekf = Extended_Kalman_Filter(pix2mm)
    ekf.Sigma = np.eye(5)
    ekf.Mu = [robot_pose_px[0][0], robot_pose_px[0][1], robot_pose_px[1], 0, 0]
    ekf.old_time = time.time()

    return cam, driver, ekf, checkpoints

def update_camera_and_kalman(cam: camera.Camera):
    global running
    # global driver
    print("Starting update_camera_and_kalman thread")
    while running:
        # Update the camera and perform filtering
        cam.update(corners=False, obstacles_goal=False, show_all=False)
        robot_pose_px = cam.get_robot_pose()

        ekf.update_time(time.time())
        l_speed, r_speed = driver.get_motor_speeds()
        # print(f"robot speed kalman: {l_speed}\t {r_speed}")
        ekf.extended_kalman(
            ekf.u_input(l_speed, r_speed),
            ekf.system_state(robot_pose_px),
        )
        robot_pose_mm = ([ekf.Mu[0], ekf.Mu[1]], ekf.Mu[2])

        # Reset filter
        ekf.Mu = [robot_pose_mm[0][0], robot_pose_mm[0][1], robot_pose_mm[1], 0, 0]
        # print(f"robot speed pose kalmam: {robot_pose_mm}")
        # Display the frame and map
        cam.display_map(robot_pose_mm)
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
    state = 0
    for i in range(len(checkpoints)):
        if DEBUG:
            print(f"Moving to checkpoint: {checkpoints[i]}")
        while not checkpoint_reached(checkpoints[i], robot_pose[0]):
            if obstacle_detected():
                state = 1
            if state == 0: # global nav
                driver.move_to_checkpoint(robot_pose, checkpoints[i])
            elif state == 1: # local nav
                # use proximity sensor inputs to avoid obstacles
                ...
            # update robot pose by vision + kalman
            robot_pose = camera.get_robot_pose()

    
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