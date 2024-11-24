import yaml
import os
import cv2
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
YAML_PATH = dir_path+"/values.yml"

class Thresholds:
    """Class used to store the parameters for the image segmentation
    
    Attributes:
        blue: Blue threshold
        green: Green threshold
        red: Red threshold
        kernel_size: Kernel size for closing
        area: Area size to filter the contours
    """
    def __init__(self, dict: dict):
        self.blue = dict["blue"]
        self.green = dict["green"]
        self.red = dict["red"]
        self.kernel_size = dict["kernel_size"]
        self.area = dict["area"]


class Hyperparameters:
    """Class used to store the hyperparameters for the vision
    
    Attributes:
        map_size: [width, height] of extracted map
        obstacles (Thresholds): parameters for obstacle extraction
        goal (Thresholds): parameters for goal extraction
    """
    def __init__(self, dict):
        self.map_size = dict["map_size"]
        self.obstacles = Thresholds(dict["obstacles"])
        self.goal = Thresholds(dict["goal"])


# Chat gpt
def object_to_dict(obj):
    if isinstance(obj, Thresholds):
        return obj.__dict__  # Convert Thresholds object to dict
    elif isinstance(obj, Hyperparameters):
        return {
            "map_size": obj.map_size,
            "obstacles": object_to_dict(obj.obstacles),
            "goal": object_to_dict(obj.goal)
        }
    else:
        return obj 
    

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
    corners = np.array(corners, dtype="float32")
    destination_points = np.array([
    [0, height - 1],
    [0, 0],
    [width - 1, 0],
    [width - 1, height - 1],
    ], dtype="float32")

    # Compute the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(corners, destination_points)

    # Perform the perspective warp
    warped = cv2.warpPerspective(img, matrix, (width, height))

    return warped


def get_corner(corners: np.ndarray) -> list:
    """
    Returns the first corner of the corners fround from an aruco detector

    Args:
        corners (np.ndarray): The 4 corners of an aruco markers returned by an aruco
                              detector
    
    Returns:
        list: The first corner of the aruco markers in format [x1,y1]
    """
    # Arcuo corners are of the format array([[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]])
    return corners[0][0].astype(int).tolist()
