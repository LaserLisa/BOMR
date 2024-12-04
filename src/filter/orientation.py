class Orientation:
    """Keeps a moving average of a orientation measurement.
    
    Attributes:
        angle: The current position of the object
        _hist: Past postions of the object
    """
    
    def __init__(self, window_size: int = 10):
        """Initializes a Postion object
        
        Args:
            window_size (int): The number of past positions to keep. Used for the 
                               rolling average.
        """
        self.value: np.ndarray = np.array([np.nan])
        self._hist: np.ndarray = np.ones((window_size,2))*np.nan


    def update(self, new_position: np.ndarray | list):
        """Calculates the angle based on the orientation vector
        
        Args:
            new_position (np.ndarray|list): new measured position
        """
        # print("new_position", new_position)
        self._hist = np.insert(self._hist, 0, new_position, axis=0)
        self._hist = self._hist[:-1]
        

        if np.isnan(self._hist).all():
            self.value = np.nan
        else:
            new_position = np.nanmean(self._hist, axis=0)
            # print("new_position", new_position)
            self.value = np.arctan2(new_position[1], -new_position[0])
            
