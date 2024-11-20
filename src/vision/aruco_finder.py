import cv2
import os
import numpy as np
from camera import Camera
import helpers
CAMERA = True
IMG_PATH = "test1.jpg"
global_corners = [None,None,None,None]

def get_4_corners(corners):
    c1 = corners[0][0][0].astype(int)
    c2 = corners[1][0][0].astype(int)
    c3 = corners[2][0][0].astype(int)
    c4 = corners[3][0][0].astype(int)
    return (c1, c2, c3, c4)

def get_corner(corners):
    return corners[0][0].astype(int).tolist()

def save_thresholds():
    thresholds = {"red": cv2.getTrackbarPos('red_channel_threshold', 'Settings'),
                  "green": cv2.getTrackbarPos('green_channel_threshold', 'Settings'),
                  "blue": cv2.getTrackbarPos('blue_channel_threshold', 'Settings'),
                  "kernel_size": cv2.getTrackbarPos('kernel_size', 'Settings'),
                  "canny1": cv2.getTrackbarPos('canny_threshold1', 'Settings'),
                  "canny2": cv2.getTrackbarPos('canny_threshold2', 'Settings')}
    helpers.dump_yaml(thresholds)


def preprocess(img, aruco_dict, parameters):

    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Create the ArUco detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)
    # Print the detected markers
    # print("Detected markers:", ids)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        for i, id in enumerate(ids):
            global_corners[id[0]-1] = get_corner(corners[i])

    if None not in global_corners:
        # draw line from corners to corners
        # cv2.line(img, corne)
        # print(corners)
        # corners = get_4_corners(corners)
        cv2.line(img, global_corners[0], global_corners[1], (0,0,255), 4)
        cv2.line(img, global_corners[1], global_corners[2], (0,0,255), 4)
        cv2.line(img, global_corners[2], global_corners[3], (0,0,255), 4)
        cv2.line(img, global_corners[3], global_corners[0], (0,0,255), 4)
                

    cv2.imshow('Detected Markers', img)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    


if __name__ == "__main__":
    # init_setting()
    if CAMERA:
        cam = Camera(1)
    else:
        img = cv2.imread(IMG_PATH)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    while True:
        if CAMERA:
            img = cam.get_current_frame()
        preprocess(img, aruco_dict, parameters)

        # wait for a key being pressed
        # check if 'q' is pressed --> quit
        if cv2.waitKey(1) == ord('q'):
            break
    # save_thresholds()



