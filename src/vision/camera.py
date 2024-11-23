import cv2
import numpy as np
from .helpers import get_corner, perspective_transform, read_yaml
from .measurements import Position

class Camera(cv2.VideoCapture):
    def __init__(self, camera=0):
        """
        Instantiates a camera
        
        Args:
            camera: opencv identifier of the camera
        """
        super().__init__(camera)
        self._frame, self.ret = None, None
        self._map = None
        self._init_map = False
        self._corners = [None, None, None, None]
        self._goal_position = None
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, 
                                                       cv2.aruco.DetectorParameters())
        self._thresholds = read_yaml()

    def read(self) -> cv2.typing.MatLike:
        self._ret, self._frame = super().read()
        return self._frame
        
    def get_current_frame(self)->np.ndarray:
        """
        Returns current frame of camera
        Args:
            None
        
        Returns:
            np.ndarray: current frame
            """
        return self._frame
    
    def initialize_map(self, show=False):
        """"
        Initializes the map by extracting the obstacles from
        """
        while None in self._corners:
            self.read()
            self._find_corners(show)
        print("corners found")
        self._extract_obstacles()
        self._init_map = True

    def update(self, show_all: bool = False):
        """Aquire new frame, find corners, extract obstacles, update goal and robot
        position.
        """
        self._ret, self._frame = self.read()
        self._find_corners()
        self._extract_obstacles(show_warped=show_all)
        self._extratct_goal(show_warped=show_all)

    def display_map(self, pose_estimation: np.ndarray|list = []):
        """Displays the map with obstacles, goal position, robot position from vision
        and estimated robot position from the Kalman filter.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        map = self._map.copy()
        cv2.circle(map, self._goal_position, 5, (255, 0, 0), cv2.FILLED)
        if pose_estimation:
            cv2.circle(map, self._goal_position, 5, (0, 0, 255), cv2.FILLED)
        cv2.imshow('map', map)



    def get_map(self)->np.ndarray:
        """
        Returns the map with the obstacles if it is initalized. Otherwise raises error.

        The maps contains '1' if there is an obstacle and '0' else.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        return cv2.cvtColor(self._map, cv2.COLOR_BGR2GRAY)
    
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
        # find aruco marker with id 42 on warped image
        # get central position
        # get angle to horizontal

    def _find_corners(self, show=False):
        """
        Detects the aruco markers of the corners and updates the position
        """
        if show:
            img = self._frame.copy()
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self._aruco_detector.detectMarkers(gray)
        if ids is not None:
            # the 4 corner of our map have ids 1,2,3,4
            for i, id in enumerate(ids):
                if id >= 1 or id <= 4:  # only consider corner markers
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
        thresholded = cv2.inRange(warped, (0, 0, 0), 
                                  (self._thresholds["blue"], self._thresholds["green"],
                                    self._thresholds["red"]))
        
        canny = cv2.Canny(thresholded, 94, 98, apertureSize=3)
        kernel = np.ones((self._thresholds["kernel_size"], 
                          self._thresholds["kernel_size"]), np.uint8)
        morph = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        # filter controus where area < 1000
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]

        map = np.zeros_like(warped, dtype=np.uint8)
        
        cv2.drawContours(map, contours, -1, (255, 255, 255), cv2.FILLED)
        if show_warped:
            cv2.drawContours(warped, contours, -1, (0, 0, 255), 1)
            cv2.imshow('warped', warped)
        self._map = map


    def _extratct_goal(self, show_warped: bool = False):
        warped = perspective_transform(self._frame, self._corners, 350, 240)
        thresholded = cv2.inRange(warped, (0, 0, 161), 
                                    (96, 126,255))
        
        canny = cv2.Canny(thresholded, 94, 98, apertureSize=3)
        kernel = np.ones((self._thresholds["kernel_size"], 
                            self._thresholds["kernel_size"]), np.uint8)
        morph = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # filter controus where area < 500
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]
        if contours:
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self._goal_position = [cx, cy]
        
        if show_warped:
            cv2.drawContours(warped, contours, -1, (0, 0, 0), 2)
            cv2.circle(warped, self._goal_position, 5, (255, 0, 0), cv2.FILLED)
            cv2.imshow('goal', warped)
        