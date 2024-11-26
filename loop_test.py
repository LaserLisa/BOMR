import cv2
import numpy as np
from src.vision import camera
import src.path_planning.path_planning as pp
from src.filter.kalman_filter import Extended_Kalman_Filter
from driving import Driving

MAP_SIZE_MM = [1050, 720]

# Open the default camera
print("Initializing camera...")
cam = camera.Camera(1)
pix2mm = MAP_SIZE_MM[0] / cam._hyperparams.map_size[0]

print("Initializing map...")
cam.initialize_map(show=True)
robot_pose_px = cam.get_robot_pose()
map_data = cam.get_map()
goal = cam.get_goal_position()
checkpoints = pp.get_checkpoints(map_data, robot_pose_px[0], goal)

print("Initialize Thymio...")
# TODO: Thymio initialization
driver = Driving()

# TODO: Initialize Kalman filter (placeholder for now)
kalman_filter = Extended_Kalman_Filter()

try:
    while True:
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

        # Update camera and process robot pose
        cam.update(show_all=False)
        robot_pose_px = cam.get_robot_pose()
        
        # Kalman filter update (example logic; modify as needed)
        robot_pose_mm = [robot_pose_px[0] * pix2mm, robot_pose_px[1] * pix2mm]
        kalman_filter.update(robot_pose_mm)

        # Display the current frame and the map
        frame = cam.get_current_frame()
        cam.display_map()
        cv2.imshow('Camera', frame)

        # Press 'q' to exit the loop (alternative exit option)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting program via 'q'.")
            break
except KeyboardInterrupt:
    print("\nProgram interrupted by user.")

finally:
    # Release the capture and writer objects
    cam.release()
    cv2.destroyAllWindows()
