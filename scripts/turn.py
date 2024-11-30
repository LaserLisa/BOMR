import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.motion_control.driving import Driving

driver = Driving()

driver.execute_command(0, 0, 0)