import os
import numpy as np
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.vision.measurements import Position, Orientation

if __name__ == "__main__":
    window_size = 5
    # dummy values with nans
    pos_t = np.array([[1,1], [1,1], [1,1], [1,1], [1,1], [np.nan, np.nan], 
                      [np.nan, np.nan], [np.nan, np.nan], [np.nan, np.nan], 
                      [np.nan, np.nan], [np.nan, np.nan], [np.nan, np.nan]])
    
    or_vec_t = np.array([[1,1.6], [1.1,1], [1,.51], [1.3,1], [1,1], [np.nan, np.nan], 
                         [np.nan, np.nan], [np.nan, np.nan], [np.nan, np.nan], 
                         [np.nan, np.nan], [np.nan, np.nan], [np.nan, np.nan]])

    pos = Position(window_size)
    orien = Orientation(window_size)

    for i in range(len(pos_t)):
        pos.update(pos_t[i])
        orien.update(or_vec_t[i])
        print(f"Time {i}:\t Position: {pos.value}, Orientation: {orien.value}")