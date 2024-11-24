import cv2
import numpy as np
from .helpers import get_corner, perspective_transform, read_yaml, Hyperparameters
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
        self._robot_position = None
        self._robot_orientation = None
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, 
                                                       cv2.aruco.DetectorParameters())
        self._hyperparams = Hyperparameters(read_yaml())

    def read(self) -> cv2.typing.MatLike:
        """Acquries new frame and returns it"""
        self._ret, self._frame = super().read()
        return self._frame
        
    def get_current_frame(self)->np.ndarray:
        """Returns current frame of camera"""
        return self._frame
    
    def initialize_map(self, show: bool = True):
        """"
        Initializes the map by extracting the obstacles from

        Args:
            show (bool): If True shows camera frame while initalization. Default True.
        """
        while None in self._corners:
            self.read()
            self._find_corners(show)
        print("corners found")
        self._extract_obstacles()
        while self._robot_position is None or self._goal_position is None:
            self._extract_goal()
            self._extract_robot_pose()
            self.read()
        self._init_map = True

    def update(self, show_all: bool = False):
        """
        Aquire new frame, find corners, extract obstacles, update goal and robot
        position.

        Args:
            show_all (bool): If True shows all intermediate windows. Default: False
        """
        self._frame = self.read()
        # self._find_corners()
        # self._extract_obstacles(show_warped=show_all)
        # self._extract_goal(show_warped=show_all)
        self._extract_robot_pose(show=True)

    def display_map(self, pose_estimation: np.ndarray|list = []):
        """
        Displays the map with obstacles, goal position, robot position from vision
        and estimated robot position from the Kalman filter.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        map = self._map.copy()
        cv2.circle(map, self._goal_position, 5, (255, 0, 0), cv2.FILLED)
        if pose_estimation:
            cv2.circle(map, (pose_estimation/3).astype(int), 5, (0, 0, 255), cv2.FILLED)
        if self._robot_position is not None:
            # (x0, y0) = self._robot_position.astype(int)
            # (x1, y1) = (self._robot_position.astype(int)+[5,0])
            # a = self._robot_orientation
            # x2 = ((x1 - x0) * np.cos(a)) - ((y1 - y0) * np.sin(a)) + x0
            # y2 = ((x1 - x0) * np.sin(a)) + ((y1 - y0) * np.cos(a)) + y0
            cv2.circle(map, self._robot_position.astype(int), 5, (0, 255, 255), cv2.FILLED)
            # cv2.arrowedLine(map, (x0, y0), (x2, y2),  
            #         (0, 255, 255), 3, tipLength = 0.5)  
        cv2.imshow('map', map)



    def get_map(self)->np.ndarray:
        """
        Returns the map with the obstacles if it is initalized. Otherwise raises error.

        The maps contains '1' if there is an obstacle and '0' else.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        return (cv2.cvtColor(self._map, cv2.COLOR_BGR2GRAY) == 255).astype(int)
    
    def get_robot_pose(self) -> tuple:
        """Returns the robot pose ([x,y],angle)"""
        return (self._robot_position, self._robot_orientation)
    
    def get_goal_position(self) -> list:
        """Returns the goal position"""
        return self._goal_position
    
    def _extract_robot_pose(self, show: bool = False):
        """
        Extracts the robot pose from latest frame
        Args:
            show (bool): If True shows the found aruco marker
        """
        warped = perspective_transform(self._frame, self._corners, 
                                       self._hyperparams.map_size[0], 
                                       self._hyperparams.map_size[1])
        if show:
            img = warped.copy()
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self._aruco_detector.detectMarkers(gray)
        if ids is not None:
            # the 4 corner of our map have ids 1,2,3,4
            for i, id in enumerate(ids):
                if id == 42:  # only consider corner markers
                    c1 = corners[0][0][0].astype(int)
                    c2 = corners[0][0][1].astype(int)
                    c3 = corners[0][0][2].astype(int)
                    center = np.mean([c1, c3], axis=0)
                    p = np.mean([c1, c2], axis=0)
                    angle = np.arctan2((center-p)[1], (p-center)[0])
                    self._robot_orientation = angle
                    self._robot_position = center
                if show:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)
            
        if show:
            if self._robot_orientation is not None:
                cv2.putText(img, str(np.degrees(self._robot_orientation).astype(int)), (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))
            cv2.imshow('Robot', img)

    def _find_corners(self, show=False):
        """
        Detects the aruco markers of the corners and updates the position

        Args:
            show (bool): If True shows the frame with drawn markers. Default: False
        """
        if show:
            img = self._frame.copy()
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self._aruco_detector.detectMarkers(gray)
        if ids is not None:
            # the 4 corner of our map have ids 1,2,3,4
            for i, id in enumerate(ids):
                if id >= 1 and id <= 4:  # only consider corner markers
                    self._corners[id[0]-1] = get_corner(corners[i])
            if show:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
        
        if show:
            cv2.imshow('Camera', img)
            cv2.waitKey(1)


    def _extract_obstacles(self, show_warped: bool = False):
        """
        Thresholds the current frame to see the black obstacles.

        Args:
            show_warped (bool): If True shows the cropped image with drawn contours. 
                                Default: False.
        """
        warped = perspective_transform(self._frame, self._corners, 
                                       self._hyperparams.map_size[0], 
                                       self._hyperparams.map_size[1])
        thresholded = cv2.inRange(warped, (0, 0, 0), 
                                  (self._hyperparams.obstacles.blue, 
                                   self._hyperparams.obstacles.green,
                                   self._hyperparams.obstacles.red))
        
        canny = cv2.Canny(thresholded, 94, 98, apertureSize=3)
        kernel = np.ones((self._hyperparams.obstacles.kernel_size, 
                          self._hyperparams.obstacles.kernel_size), np.uint8)
        morph = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        # filter controus where area < 1000
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self._hyperparams.obstacles.area]

        map = np.zeros_like(warped, dtype=np.uint8)
        
        cv2.drawContours(map, contours, -1, (255, 255, 255), cv2.FILLED)
        if show_warped:
            cv2.drawContours(warped, contours, -1, (0, 0, 255), 1)
            cv2.imshow('warped', warped)
        self._map = map


    def _extract_goal(self, show_warped: bool = False):
        """
        Thresholds the current frame to extract the goal position.

        Args:
            show_warped (bool): If True shows the cropped image with drawn contour. 
                                Default: False.
        """
        warped = perspective_transform(self._frame, self._corners, 
                                       self._hyperparams.map_size[0], 
                                       self._hyperparams.map_size[1])
        thresholded = cv2.inRange(warped, (0, 0, self._hyperparams.goal.red), 
                                    (self._hyperparams.goal.blue, 
                                     self._hyperparams.goal.green,255))
        
        canny = cv2.Canny(thresholded, 94, 98, apertureSize=3)
        kernel = np.ones((self._hyperparams.goal.kernel_size, 
                          self._hyperparams.goal.kernel_size), np.uint8)
        morph = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # filter controus where area < 500
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self._hyperparams.goal.area]
        if contours:
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self._goal_position = [cx, cy]
        
        if show_warped:
            cv2.drawContours(warped, contours, -1, (0, 0, 0), 2)
            cv2.circle(warped, self._goal_position, 5, (255, 0, 0), cv2.FILLED)
            cv2.imshow('goal', warped)
        