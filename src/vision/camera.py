import cv2
import numpy as np

class Camera(cv2.VideoCapture):
    def __init__(self, camera=0):
        """
        Instantiates a camers
        
        Args:
            camera: opencv identifier of the camera
        """
        super().__init__(camera)
        self._frame, self.ret = None, None
        self._map = None
        self._init_map = False
        
    def get_current_frame(self)->np.ndarray:
        """
        Returns current frame of camera
        Args:
            None
        
        Returns:
            np.ndarray: current frame
            """
        self._ret, self._frame = self.read()
        return self._frame
    
    def initialize_map(self):
        """"
        Initializes the map by extracting the obstacles from
        """
        # TODO: extract map containing obstacles: rectify map, map pixels to map size
        # and extract obstacles 
        self._init_map = True

    def get_map(self)->np.ndarray:
        """
        Returns the map with the obstacles if it is initalized. Otherwise raises error.

        The maps contains '1' if there is an obstacle and '0' else.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        return self._map
    
    def get_robot_pose(self)->tuple:
        """
        Extracts the robot pose from latest frame
        Args:
            None
        Returns:
            tuple(int, int, float): tuple with x and y coordinates of the robot 
                                    position and the angle of rotation.
        """
        ...
        
    