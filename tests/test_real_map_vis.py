import cv2
import numpy as np
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vision import camera

# Open the default camera
print("Initalizing camera...")
cam = camera.Camera(0, window_size=2)
pix2mm = cam.pixel2mm

print("Intializing map...")
cam.initialize_map(show=True, show_all=False)

while True:
    # to display the current frame and the map
    frame = cam.read()
    cam.update(corners=False, obstacles_goal=False)
    cam.display_map()
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break
# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()

