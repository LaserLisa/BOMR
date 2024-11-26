import cv2
import numpy as np
from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from driving import Driving

MAP_SIZE_MM = [1050, 720]
# Open the default camera
print("Initalizing camera...")
cam = camera.Camera(1)
pix2mm = MAP_SIZE_MM[0]/cam._hyperparams.map_size[0]

print("Intializing map...")
cam.initialize_map(show=True)
robot_pose_px = cam.get_robot_pose()
map = cam.get_map()
goal = cam.get_goal_position()
checkpoints = pp.get_checkpoints(map, robot_pose_px[0], goal)

print("Initalize Thymio...")
# TODO: Thymio initalization
driver = Driving()
# TODO: initialize kalman filter
while True:
    # if obstacle_detected():
    #    ...
    
    # else:
    # move towards next checkpoint
    # Get user input for the command
    command = input("Enter command ('turn degrees' or 'move duration') or 'exit' to quit: ").strip()
    if command.lower() == "exit":
        print("Exiting program.")
        break

    # Parse the command
    try:
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
    except ValueError:
        print("Invalid input. Please ensure the correct format for 'turn degrees' or 'move duration'.")

    # get robot pose
    cam.update(show_all=False)
    robot_pose_px = cam.get_robot_pose()

    # kalman filter
    robot_pose_mm = ...

    # to display the current frame and the map
    frame = cam.get_current_frame()
    cam.display_map()
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break
# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()
