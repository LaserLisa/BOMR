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
    
    def initialize_map():
        """"
        Initializes the map by extracting the obstacles from
        """
        ...
        
    
        
    