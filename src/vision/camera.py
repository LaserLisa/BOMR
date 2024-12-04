import cv2
import numpy as np
import time
from .helpers import get_corner, perspective_transform, read_yaml, Hyperparameters
from .measurements import Position, Orientation

class Camera(cv2.VideoCapture):
    def __init__(self, camera=0, window_size=10):
        """
        Instantiates a camera
        
        Args:
            camera: opencv identifier of the camera
        """
        super().__init__(camera)
        self._frame, self.ret = None, None
        self._warped = None
        self._map = None
        self._obstacles_contours = None
        self._window_size = window_size
        self._init_map = False
        self._corners = [None, None, None, None]
        self._goal_position = None
        self._robot_position = Position(window_size)
        self._robot_orientation = Orientation(window_size)
        self._checkpoints = None
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, 
                                                       cv2.aruco.DetectorParameters())
        self._hyperparams = Hyperparameters(read_yaml())
        self.pixel2mm = self._hyperparams.map_size_mm[0]/self._hyperparams.map_size[0]

    def read(self) -> cv2.typing.MatLike:
        """Acquries new frame and returns it"""
        self._ret, self._frame = super().read()
        return self._frame
        
    def get_current_frame(self)->np.ndarray:
        """Returns current frame of camera"""
        return self._frame
    
    def initialize_map(self, show: bool = True, show_all: bool = False):
        """"
        Initializes the map by extracting the obstacles from

        Args:
            show (bool): If True shows camera frame while initalization. Default True.
        """
        t_start = time.time()
        # let the camera adjust to the light
        print("\t\t>>>initialize_map():\tadjusting to light...")
        while time.time() - t_start < 1:
            self.read()
            cv2.waitKey(1)
        while None in self._corners:
            self.read()
            self._find_corners(show)
        self._warped = perspective_transform(self._frame, self._corners,
                                            self._hyperparams.map_size[0],
                                            self._hyperparams.map_size[1])
        print("\t\t>>>initialize_map():\tcorners found, extracting obstacles...")
        self._extract_obstacles(show_all)
        print("\t\t>>>initialize_map():\textracting robot and goal position...")
        while np.isnan(self._robot_position.value).all() or self._goal_position is None:
            self._extract_goal(show_all)
            self._extract_robot_pose(show_all)
            self.read()
            self._warped = perspective_transform(self._frame, self._corners,
                                            self._hyperparams.map_size[0],
                                            self._hyperparams.map_size[1])
            if show:
                cv2.imshow('Camera', self._frame)
                cv2.waitKey(1)
        cv2.destroyWindow("Camera")
        self._init_map = True

    def update(self, corners: bool, obstacles_goal: bool, show_all: bool = False):
        """
        Aquire new frame and updates the robot postion. Optionally also updates the 
        corners, obstacles and goal position.

        Args:
            corners (bool): If True relocates the corners.
            obstacles_goal (bool): If True updates obstacles and goal position.
            show_all (bool): If True shows all intermediate windows. Default: False
        """
        self._frame = self.read()
        if corners:
            self._find_corners()
        self._warped = perspective_transform(self._frame, self._corners,
                                            self._hyperparams.map_size[0],
                                            self._hyperparams.map_size[1])
        if obstacles_goal:
            self._extract_obstacles(show_warped=show_all)
            self._extract_goal(show_warped=show_all)
        self._extract_robot_pose(show=show_all)

    def display_map(self, pose_estimation: tuple = None, alpha: float = 0.7):
        """
        Displays the map with obstacles, goal position, robot position from vision
        and estimated robot position from the Kalman filter.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        map = self._warped.copy()
        overlay = map.copy()
        # draw goal position
        cv2.circle(overlay, self._goal_position, 5, (255, 0, 0), cv2.FILLED)
        # draw obstacles
        cv2.drawContours(overlay, self._obstacles_contours, -1, (255, 255, 255), cv2.FILLED)
        def draw_robot(position, angle, color):
            # 80mm is the distance from center of rotation to front of robot
            radius = int(80/self.pixel2mm) 
            position = position.astype(int)
            cv2.circle(overlay, position, radius, color, 2)
            arrow_start_x = int(position[0])
            arrow_start_y = int(position[1])
            arrow_end_x = int(arrow_start_x + (10+radius) * np.cos(angle))
            arrow_end_y = int(arrow_start_y - (10+radius) * np.sin(angle))
            cv2.arrowedLine(overlay, (arrow_start_x, arrow_start_y), 
                            (arrow_end_x, arrow_end_y), color, 2)
        
        def draw_path(checkpoints):
            # Draw checkpoints (as circles) on the image
            for checkpoint in checkpoints:
                cv2.circle(overlay, checkpoint, 5, (0, 0, 255), -1)  # Red circles for checkpoints

            # Draw lines from checkpoint to checkpoint
            for i in range(len(checkpoints) - 1):
                start_point = checkpoints[i]
                end_point = checkpoints[i + 1]
                cv2.line(overlay, start_point, end_point, (0, 255, 0), 2)  # Green line
        
        if self._checkpoints:
            draw_path(self._checkpoints)

        if pose_estimation:
            if not (np.isnan(pose_estimation[0]).any() or np.isnan(pose_estimation[1])):
                draw_robot(np.array(pose_estimation[0]), pose_estimation[1], (0, 255, 0))
        if not (np.isnan(self._robot_position.value).any() or 
                np.isnan(self._robot_orientation.value)):
            position = self._robot_position.value.astype(int)
            angle = self._robot_orientation.value
            draw_robot(position, angle, (0, 255, 255))
        map = cv2.addWeighted(overlay, alpha, map, 1-alpha, 0)
        # resize map
        map = cv2.resize(map, (0, 0), fx=2, fy=2)
        cv2.imshow('map', map)



    def get_map(self)->np.ndarray:
        """
        Returns the map with the obstacles if it is initalized. Otherwise raises error.

        The maps contains '1' if there is an obstacle and '0' else.
        """
        assert self._init_map, "Map not initalized, call cam.initalize_map() first."
        return (cv2.cvtColor(self._map, cv2.COLOR_BGR2GRAY) == 255).astype(int)
    
    def reset(self):
        """Resets the map"""
        self._init_map = False
        self._corners = [None, None, None, None]
        self._goal_position = None
        self._robot_position = Position(self._window_size)
        self._robot_orientation = Orientation(self._window_size)
        self._checkpoints = None
    
    def get_robot_pose(self) -> tuple:
        """Returns the robot pose ([x,y],angle)"""
        return (self._robot_position.value, self._robot_orientation.value)
    
    def get_goal_position(self) -> list:
        """Returns the goal position"""
        return self._goal_position
    
    def set_checkpoints(self, points):
        """Sets the list of checkpoints"""
        self._checkpoints = points

    def _extract_robot_pose(self, show: bool = False):
        """
        Extracts the robot pose from latest frame
        Args:
            show (bool): If True shows the found aruco marker
        """
        warped = self._warped.copy() 
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
                    # angle = np.arctan2((center-p)[1], (p-center)[0])
                    self._robot_orientation.update(center-p)
                    self._robot_position.update(center)
                if show:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)
        else:
            # if no aruco marker found update with nan
            self._robot_position.update([np.nan, np.nan])
            self._robot_orientation.update([np.nan, np.nan])
            
        if show:
            if not np.isnan(self._robot_orientation.value):
                cv2.putText(img, str(np.degrees(self._robot_orientation.value).astype(int)), (50, 50), 
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
        warped = self._warped.copy()
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
        self._obstacles_contours = contours
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
        warped = self._warped.copy()
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
            cv2.waitKey(1)