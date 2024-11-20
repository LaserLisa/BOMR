import yaml
import os
import cv2
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
YAML_PATH = dir_path+"/values.yml"

def pixel2cell(x: int, y: int) -> tuple[int, int]:
    """
    Converts from the camera pixel space into the map space
    
    Args:
        x (int): the x coordinate of the camera pixel
        y (int): the y coordinate of the camera pixel
        
    Returns:
        tuple(r,c): corresponding row (r) and column (c) in the map space
    """
    ...
    
def cell2pixel(r: int, c: int)-> tuple[int, int]:
    """
    Converts from the map space into the camer pixel space
    
    Args:
        r (int): the x coordinate of the camera pixel
        c (int): the y coordinate of the camera pixel
        
    Returns:
        tuple(x,y): corresponding x and y coodinate in the pixel space
    """
    ...


def dump_yaml(data):
    with open(YAML_PATH, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

# https://stackoverflow.com/questions/1773805/how-can-i-parse-a-yaml-file-in-python
def read_yaml():
    with open(YAML_PATH) as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

def perspective_transform(img: np.ndarray, corners: np.ndarray, width, height) -> np.ndarray:
    """
    Rectifies quadrat formed by four corner points into an image of width x height.
    
    Args:
        img (np.ndarray): The original image
        corners (np.ndarray): The four corner points in img
        width: width of the new image
        height: height of the new image
    Returns:
        np.ndarray: The transformed image
    """
    destination_points = np.array([
    [0, 0],
    [width - 1, 0],
    [0, height - 1],
    [width - 1, height - 1]
    ], dtype="float32")

    # Compute the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(corners, destination_points)

    # Perform the perspective warp
    warped = cv2.warpPerspective(img, matrix, (width, height))

    return warped
