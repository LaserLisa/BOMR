import cv2
import os
import numpy as np
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vision.camera import Camera
from src.vision.helpers import dump_yaml, read_yaml
CAMERA = True
IMG_PATH = "test_map.jpg"

def init_setting():
    thresholds = read_yaml()
    cv2.namedWindow("Settings")
    cv2.createTrackbar("red_channel_threshold", "Settings", thresholds["red"], 255, 
                       lambda x: None)
    cv2.createTrackbar('green_channel_threshold', 'Settings', thresholds["green"], 255, 
                       lambda x: None)
    cv2.createTrackbar("blue_channel_threshold", "Settings", thresholds["blue"], 255, 
                       lambda x: None)
    cv2.createTrackbar('kernel_size', 'Settings', thresholds["kernel_size"], 40, 
                       lambda x: None)
    cv2.createTrackbar('canny_threshold1', 'Settings', thresholds["canny1"], 255, 
                       lambda x: None)
    cv2.createTrackbar('canny_threshold2', 'Settings', thresholds["canny2"], 255, 
                       lambda x: None)

def save_thresholds():
    thresholds = {"red": cv2.getTrackbarPos('red_channel_threshold', 'Settings'),
                  "green": cv2.getTrackbarPos('green_channel_threshold', 'Settings'),
                  "blue": cv2.getTrackbarPos('blue_channel_threshold', 'Settings'),
                  "kernel_size": cv2.getTrackbarPos('kernel_size', 'Settings'),
                  "canny1": cv2.getTrackbarPos('canny_threshold1', 'Settings'),
                  "canny2": cv2.getTrackbarPos('canny_threshold2', 'Settings')}
    dump_yaml(thresholds)

def preprocess(img, obstacles = True):
    red = cv2.getTrackbarPos('red_channel_threshold', 'Settings')
    green = cv2.getTrackbarPos('green_channel_threshold', 'Settings')
    blue = cv2.getTrackbarPos('blue_channel_threshold', 'Settings')
    kernel_size = cv2.getTrackbarPos('kernel_size', 'Settings')
    canny_threshold1 = cv2.getTrackbarPos('canny_threshold1', 'Settings')
    canny_threshold2 = cv2.getTrackbarPos('canny_threshold2', 'Settings')
    
    x = img

    orig = x.copy()
    if CAMERA:
        cv2.imwrite("testair.jpg", orig)


    # apply rgb thresholds
    # x = cv2.inRange(x, (blue, green, red), (255, 255, 255))
    x = cv2.inRange(x, (0, 0, red), (blue, green, 255))

    thresholded = x.copy()

    # # morphological operations
    # if kernel_size > 0:
    #     kernel = np.ones((kernel_size, kernel_size), np.uint8)
    #     x = cv2.morphologyEx(x, cv2.MORPH_CLOSE, kernel, iterations=2)
    #     x = cv2.morphologyEx(x, cv2.MORPH_OPEN, kernel, iterations=2)

    # # # # # # remove small objects
    # # x = cv2.erode(x, np.ones((25,25), np.uint8), iterations =1)
    # # x = cv2.dilate(x, np.ones((25,25), np.uint8), iterations=1)
    # x = cv2.GaussianBlur(x, (5, 5), 0)


    binary = x.copy()

    x = cv2.Canny(x, canny_threshold1, canny_threshold2, apertureSize=3)

    if kernel_size > 0:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        x = cv2.morphologyEx(x, cv2.MORPH_CLOSE, kernel, iterations=2)
    canny = x.copy()
    
    # # find contours
    contours, _ = cv2.findContours(x, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # filter controus where area < 500
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
    
    # print(f"number of contours: {len(contours)}\r")
    # print area of contours
    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    for i, cnt in enumerate(contours):
        # contours_poly[i] = cv2.approxPolyDP(cnt, 3, True)
        # boundRect[i] = cv2.boundingRect(contours_poly[i])
        # centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
        # cv2.rectangle(orig, (int(boundRect[i][0]), int(boundRect[i][1])), \
        #   (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), (100,100,100), 2)

        cv2.putText(orig, str(int(cv2.contourArea(cnt))), cnt[0][0], 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))

    # draw contours
    orig = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
    cv2.drawContours(orig, contours, -1, (0, 0, 255), 2)
    
    # display binary, canny, orig, thresholded in one window,
    binary = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
    thresholded = cv2.cvtColor(thresholded, cv2.COLOR_GRAY2BGR)
    orig = cv2.cvtColor(orig, cv2.COLOR_RGB2BGR)

    img_stack = np.hstack([np.vstack([binary, canny]), np.vstack([orig, thresholded])])
    # img_stack = np.hstack([x, binary, thresholded])
    cv2.imshow('binary, canny, orig, thresholded', img_stack)
    # cv2.imshow('canny, binary, thresholded', img_stack)


if __name__ == "__main__":
    init_setting()
    if CAMERA:
        cam = Camera(1)
    else:
        img = cv2.imread(IMG_PATH)
    while True:
        if CAMERA:
            img = cam.read()
        preprocess(img)

        # wait for a key being pressed
        # check if 'q' is pressed --> quit
        # cv2.waitKey()
        if cv2.waitKey(1) == ord('q'):
            break
    save_thresholds()



