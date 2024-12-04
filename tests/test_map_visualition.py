import cv2
import numpy as np
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vision import camera, helpers
from src.vision.measurements import Position, Orientation

MAP_SIZE_MM = [1050, 720]

height, width = 240, 350
destination_points = np.array([
    [0, height - 1],
    [0, 0],
    [width - 1, 0],
    [width - 1, height - 1],
    ], dtype="float32")
# Open the default camera
print("Initalizing camera...")
cam = camera.Camera(0, window_size=2)
pix2mm = MAP_SIZE_MM[0]/cam._hyperparams.map_size[0]

print("Intializing map...")
cam._corners = destination_points
cam.read()
cam._warped = helpers.perspective_transform(cam._frame, cam._corners,
                                            cam._hyperparams.map_size[0],
                                            cam._hyperparams.map_size[1])
cam._robot_position = Position()
cam._robot_orientation = Orientation()
cam._robot_position.value = np.array([int(width/2), int(height/2)])
cam._robot_orientation.value = np.pi/2
cam._map = np.zeros((height, width, 3), dtype=np.uint8)
cam._init_map = True
cam._goal_position = [int(width/3), int(height/5)]


while True:
    # to display the current frame and the map
    frame = cam.read()
    cam.display_map(pose_estimation=(np.array([int(width/2)-30, int(height/2)-30]), np.pi/3))
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break
# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()