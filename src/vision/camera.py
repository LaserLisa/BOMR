import cv2
import numpy as np
from helpers import get_corner, perspective_transform, read_yaml

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
        self._corners = [None, None, None, None]
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, 
                                                       cv2.aruco.DetectorParameters())
        self._thresholds = read_yaml()
        
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
    
    def initialize_map(self, show=False):
        """"
        Initializes the map by extracting the obstacles from
        """
        while None in self._corners:
            self.get_current_frame()
            self._find_corners(show)
        print("corners found")
        # TODO: Find way to initalize and display map at same time
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
        # find aruco marker with id 42
        # get central position
        # get angle to horizontal
        # transform central position to map frame

    def _find_corners(self, show=False):
        """
        Detects the aruco markers of the corners and updates the position
        """
        if show:
            img = self._frame.copy()
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self._aruco_detector.detectMarkers(gray)
        if ids is not None:
            # update the corner position
            # the 4 corner of our map have ids 1,2,3,4
            for i, id in enumerate(ids):
                self._corners[id[0]-1] = get_corner(corners[i])
            if show:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        if show:
            cv2.imshow('Camera', img)
            cv2.waitKey(1)



    def _extract_obstacles(self, show_warped: bool = False):
        """
        Thresholds the current frame to see the black obstacles.
        """
        warped = perspective_transform(self._frame, self._corners, 350, 240)
        # warped = self._frame.copy()
        thresholded = cv2.inRange(warped, (0, 0, 0), 
                                  (self._thresholds["blue"], self._thresholds["green"],
                                    self._thresholds["red"]))
        
        canny = cv2.Canny(thresholded, 94, 98, apertureSize=3)
        kernel = np.ones((self._thresholds["kernel_size"], 
                          self._thresholds["kernel_size"]), np.uint8)
        morph = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        # filter controus where area < 500
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]

        map = np.zeros_like(warped, dtype=np.uint8)
        
        cv2.drawContours(map, contours, -1, (255, 255, 255), cv2.FILLED)
        if show_warped:
            cv2.drawContours(warped, contours, -1, (0, 0, 255), 1)
            cv2.imshow('warped', warped)
            # cv2.imshow("thresholded", thresholded)
        cv2.imshow('map', cv2.cvtColor(map, cv2.COLOR_BGR2GRAY))

