import cv2
import numpy as np
from src.vision import camera


# Open the default camera
print("Initalizing camera...")
cam = camera.Camera(1)

print("Intializing map...")
cam.initialize_map(show=True)
while True:
    # update goal position
    cam.update(show_all=False)
    frame = cam.get_current_frame()
    cam.display_map()

    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break
# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()
