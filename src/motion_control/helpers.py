import numpy as np

def checkpoint_reached(checkpoint: np.ndarray, robot_pos: np.ndarray, 
                       eps: float = 7.5) -> bool:
    """
    Checks if the robot has reached the checkpoint.

    Args:
        checkpoint(np.ndarray): The checkpoint to reach.
        robot_pose(np.ndarray): The current pose of the robot.
        eps(float): The maximum distance in px from the checkpoint to consider it 
                    reached. Default is 7.5.
    """
    return np.linalg.norm(checkpoint - robot_pos) < eps
