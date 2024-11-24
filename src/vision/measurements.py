import numpy as np

class Position:
    """Keeps a moving average of a position measurement.
    
    Attributes:
        pos: The current position of the object
        _hist: Past postions of the object
    """
    
    def __init__(self, window_size: int = 10):
        """Initializes a Postion object
        
        Args:
            window_size (int): The number of past positions to keep. Used for the 
                               rolling average.
        """
        self.pos = np.array([np.nan, np.nan])
        self._hist = np.ones((window_size,2))*np.nan

    def update(self, new_position: np.ndarray | list):
        """Adds new measurement to measurements history and updates rolling average.
        
        Args:
            new_position (np.ndarray|list): new measured position
        """
        self._hist = np.insert(self._hist, 0, new_position)
        self._hist = self._hist[:-1]

        self.pos = np.nanmean(self._hist)