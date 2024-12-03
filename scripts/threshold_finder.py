import cv2
import os
import numpy as np
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vision.camera import Camera
from src.vision.helpers import dump_yaml, read_yaml, Hyperparameters, Thresholds, object_to_dict
CAMERA = True
OBSTACLES = True
DEBUG = False
IMG_PATH = "test_map.jpg"

def init_setting(param: Thresholds):
    # create window with fixed size
    cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Settings", 500, 60)
    cv2.createTrackbar("red_channel_threshold", "Settings", param.red, 255, 
                       lambda x: None)
    cv2.createTrackbar('green_channel_threshold', 'Settings', param.green, 255, 
                       lambda x: None)
    cv2.createTrackbar("blue_channel_threshold", "Settings", param.blue, 255, 
                       lambda x: None)
    cv2.createTrackbar('kernel_size', 'Settings', param.kernel_size, 40, 
                       lambda x: None)
    cv2.createTrackbar('area', 'Settings', param.area, 4000, 
                       lambda x: None)

def save_thresholds(hyperparams: Hyperparameters):
    dump_yaml(object_to_dict(hyperparams))

def preprocess(img, params: Thresholds, obstacles: bool = True) -> Thresholds:
    params.red = cv2.getTrackbarPos('red_channel_threshold', 'Settings')
    params.green = cv2.getTrackbarPos('green_channel_threshold', 'Settings')
    params.blue = cv2.getTrackbarPos('blue_channel_threshold', 'Settings')
    params.kernel_size = cv2.getTrackbarPos('kernel_size', 'Settings')
    params.area = cv2.getTrackbarPos('area', 'Settings')
    
    x = img

    orig = x.copy()
    if CAMERA and DEBUG:
        cv2.imwrite("testair.jpg", orig)


    # apply rgb thresholds
    if obstacles:
        x = cv2.inRange(x, (0, 0, 0), (params.blue, params.green, params.red))
    else:
        x = cv2.inRange(x, (0, 0, params.red), (params.blue, params.green, 255))

    binary = x.copy()

    x = cv2.Canny(x, 90, 90, apertureSize=3)

    if params.kernel_size > 0:
        kernel = np.ones((params.kernel_size, params.kernel_size), np.uint8)
        x = cv2.morphologyEx(x, cv2.MORPH_CLOSE, kernel, iterations=2)
    canny = x.copy()
    
    # find contours
    contours, _ = cv2.findContours(x, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # filter controus by area
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > params.area]
    
    for i, cnt in enumerate(contours):

        cv2.putText(orig, str(int(cv2.contourArea(cnt))), cnt[0][0], 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))

    # draw contours
    orig = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
    cv2.drawContours(orig, contours, -1, (0, 0, 255), 2)
    
    # display binary, canny, orig, thresholded in one window,
    binary = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
    orig = cv2.cvtColor(orig, cv2.COLOR_RGB2BGR)

    # name images
    cv2.putText(binary, "Binary", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
    cv2.putText(canny, "Canny", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
    cv2.putText(orig, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    # resize images
    binary = cv2.resize(binary, (0, 0), fx=0.5, fy=0.5)
    canny = cv2.resize(canny, (0, 0), fx=0.5, fy=0.5)
    orig = cv2.resize(orig, (0, 0), fx=0.5, fy=0.5)

    img_stack = np.hstack([np.vstack([binary, canny]), np.vstack([orig, np.zeros_like(orig)])])
    cv2.imshow('Processed Images --- Press "q" to quit', img_stack)
    return params

if __name__ == "__main__":
    parameters = Hyperparameters(read_yaml())
    if OBSTACLES:
        thresh = parameters.obstacles
    else:
        thresh = parameters.goal
    init_setting(thresh)
    if CAMERA:
        cam = Camera(0)
    else:
        img = cv2.imread(IMG_PATH)
    while True:
        if CAMERA:
            img = cam.read()
        if OBSTACLES:
            parameters.obstacles = preprocess(img, thresh, obstacles=OBSTACLES)
        else:
            parameters.goal = preprocess(img, thresh, obstacles=OBSTACLES)
        # check if 'q' is pressed --> quit
        if cv2.waitKey(1) == ord('q'):
            break
    save_thresholds(parameters)



