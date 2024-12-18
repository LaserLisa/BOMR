import cv2
import numpy as np
import time
import threading
import os
import asyncio

from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from src.motion_control.driving import Driving
from src.motion_control.helpers import checkpoint_reached
import src.motion_control.local_navigation as ln

running = False        # global variable to enable disable threads
DEBUG = True          # global variable to enable/disable debug messages



def init() -> tuple[camera.Camera, Driving, Extended_Kalman_Filter]:
    """Initializes the camera, Thymio and the Extended Kalman Filter"""
    print("Initializing camera...")
    cam = camera.Camera(0, window_size=2)
    pix2mm = cam.pixel2mm
    print(pix2mm)

    checkpoints = init_map(cam)

    print("Initializing Thymio...")
    driver = Driving()

    print("Initializing filter")
    ekf = Extended_Kalman_Filter(pix2mm, cam.get_robot_pose(), time.time())
    print(cam.get_robot_pose())
    

    return cam, driver, ekf, checkpoints

def init_map(cam: camera.Camera) -> list:
    """"Initializes the map and returns the checkpoints of the path"""
    print("Initializing map...")
    cam.initialize_map(show=True, show_all=False)
    
    robot_pose_px = cam.get_robot_pose()
    map = cam.get_map()
    goal = cam.get_goal_position()
    print("Getting checkpoints...")
    checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal, cam.pixel2mm, 
                                     plot=False)
    print(checkpoints)
    cam.set_checkpoints(checkpoints)
    return checkpoints

def recalculate_checkpoints(cam: camera.Camera, driver: Driving) -> list:
    """ Waits for the robot to be placed on the ground and recalculates the checkpoints"""
    print("Place robot again on ground...")
    robot_pose_px = cam.get_robot_pose()
    ground = asyncio.run(driver.get_prox_ground())
    while np.isnan(robot_pose_px[0]).any() or (ground[0] == 0 or ground[1] == 0):
        robot_pose_px = cam.get_robot_pose()
        ground = asyncio.run(driver.get_prox_ground())
    map = cam.get_map()
    goal = cam.get_goal_position()
    print("Getting checkpoints...")
    checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal, cam.pixel2mm, 
                                     plot=False)
    cam.set_checkpoints(checkpoints)
    return checkpoints

def update_camera_and_kalman(cam: camera.Camera):
    ekf.update_time(time.time())
    global running
    # global driver
    print("Starting update_camera_and_kalman thread")
    while running:
        # Update the camera and perform filtering
        cam.update(corners=False, obstacles_goal=False, show_all=False)
        robot_pose_px = cam.get_robot_pose()

        # TODO: To get real speeds
        # l_speed, r_speed= driver.get_motor_speeds()
        # dt = time.time()
        l_speed, r_speed, dt = driver.get_l_speeds(), driver.get_r_speeds() , time.time()


        # # print(f"robot speed kalman: {l_speed}\t {r_speed}")
        # #print("Filtering")
        robot_pose_kalman = ekf.Kalman_main(l_speed, r_speed, dt, robot_pose_px)
        # #if l_speed == -r_speed:
        # print("before kalman:", robot_pose_px)
        # print("after kalman", robot_pose_kalman)
       
        # Display the frame and map
        cam.display_map(pose_estimation=robot_pose_kalman)
        if DEBUG:
            frame = cam.get_current_frame()
            cv2.imshow("Camera", frame)
        # This function waits fro 1 ms and is necessary for the frame to be displayed
        cv2.waitKey(1) 


def motion_control(driver: Driving, camera: camera.Camera, checkpoints: list, 
                   ekf: Extended_Kalman_Filter):
    global running
    print("Starting motion_control thread")
    while True:
        robot_pose = camera.get_robot_pose()
        state = 0
        for i in range(len(checkpoints)):
            if DEBUG:
                print(f"Moving to checkpoint: {checkpoints[i]}")
            while not checkpoint_reached(checkpoints[i], robot_pose[0]):
                start_time = time.time()
                # Check if the navigation state should be switched
                state, obst = ln.get_navigation_state(driver, state)

                if DEBUG:
                    print(f"State: {state}\r", end="")
                if state == 0: #global navigation
                    driver.move_to_checkpoint(robot_pose, checkpoints[i])
                elif state ==1: #local navigation
                    motor_left_speed, motor_right_speed = ln.calculate_new_motors_speed(obst)
                    driver.set_motor_speeds(motor_left_speed, motor_right_speed)
                elif state == 2:
                    driver.stop()
                    break

                # update robot pose by vision + kalman
                robot_pose = ekf.get_robot_pose()
                print(f"time elapsed: {time.time() - start_time}\r", end="")
            if DEBUG:
                print(f"Checkpoint {i} reached")

        if state == 2:
            print("Kidnapping")
            checkpoints = recalculate_checkpoints(camera, driver)
        else:
            break


    driver.stop()
    print("Goal reached")
    
    # variable to stop the vision thread
    running = False

if __name__ == "__main__":
    running = True
    cam, driver, ekf, checkpoints = init()
    
    while True:
        # Start threads
        t1 = threading.Thread(target=update_camera_and_kalman, args=(cam, ), daemon=True)
        t2 = threading.Thread(target=motion_control, args=(driver, cam, checkpoints, ekf), daemon=True)

        print("threads defined")
        t1.start()
        t2.start()

        # wait until threads terminate
        t1.join()
        t2.join()
        cv2.destroyAllWindows()
        command = input("Press 'q' to quit and r to restart: ").strip()
        if command.lower() == "q":
            break
        elif command.lower() == "r":
            running = True
            cam.reset()
            checkpoints = init_map(cam)
            print("Restarting threads")
        else:
            print("Invalid command")
            break


    print("Releasing all objects")
    cam.release()
    print("Camera released")
    driver.__del__()
    print("Driver released")
    os.kill(os.getpid(), 9)
    
