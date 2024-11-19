import cv2
import numpy as np
from src.vision import camera


# Open the default camera
cam = camera.Camera(1)


while True:
    # ret, frame = cam.read()

    # Write the frame to the output file
    frame = cam.get_current_frame()
    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()
