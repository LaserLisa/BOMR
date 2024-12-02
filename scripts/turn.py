import sys
import os
import time
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.motion_control.driving import Driving

driver = Driving()

checkpoint = np.array([80,60])
robot_pose = (np.array([40, 50]), 0)

driver.move_to_checkpoint(robot_pose, checkpoint)
time.sleep(2)
driver.stop()