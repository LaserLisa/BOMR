from path_planning import get_global_path
import numpy as np

# This code is adapted to the map that has coordinates in the format (rows, cols)

def run_local_navigation(map, current_pose, next_checkpoints, goal, mode="global"):
    """
    Processes navigation incrementally for the given checkpoints.

    Args:
        map: The navigation map (NumPy array).
        current_pose: The current robot position (row, col).
        next_checkpoints: The remaining checkpoints to process.
        goal: The final goal position (row, col).
        mode: The navigation mode ('local' or 'global').

    Returns:
        Tuple containing updated (current_pose, next_checkpoints).
    """
    for checkpoint in next_checkpoints: 
        # Check if the all the point coordinates connecting the current_pose and the next checkpoint is only composed of non-obstacle cells
        if not is_path_clear(map, current_pose, checkpoint):
            print("New obstacle detected!")
            if mode == "local":
                print("Switching to local navigation mode...")
                # Contourn the obstacle until a clear path between the current pose and one of the next checkpoints is found
                next_checkpoints = get_local_path(map, current_pose, checkpoint)
                print("The new path is:", next_checkpoints) 
                break
            else: 
                print("Recomputing the global navigation algorithm...")
                # Recompute the path using A* from the current_pose to the goal
                next_checkpoints = get_global_path(map, current_pose, goal)
                print("The new path is:", next_checkpoints)
                break
        else:
            # Move towards the next checkpoint
            print("No path recomputing needed => Moving to the next checkpoint:", checkpoint)
            # TODO: Add later the function to move to next checkpoint using the controller
            current_pose[:] = checkpoint
            next_checkpoints.pop(0)  # Remove the checkpoint after reaching it
            break  # Process one step at a time
    
    return current_pose, next_checkpoints

      
def is_path_clear(map, current_pose, checkpoint):
    # 1. Get the path from the current_pose to the checkpoint
    path = get_path(current_pose, checkpoint)

    # 2. Check if all the point coordinates in the path are only composed of non-obstacle cells
    for point in path:
        if map[point[0], point[1]] == 1:
            return False

    return True

def get_path(current_pose, checkpoint):
    # Get the direction from current_pose to checkpoint
    direction = get_direction(current_pose, checkpoint)

    # Generate the path between current_pose and checkpoint
    path = []
    if direction == "up":
        for i in range(current_pose[0] - 1, checkpoint[0] - 1, -1):  # Moving up in y-axis
            path.append((i, current_pose[1]))
    elif direction == "down":
        for i in range(current_pose[0] + 1, checkpoint[0] + 1):  # Moving down in y-axis
            path.append((i, current_pose[1]))
    elif direction == "left":
        for i in range(current_pose[1] - 1, checkpoint[1] - 1, -1):  # Moving left in x-axis
            path.append((current_pose[0], i))
    elif direction == "right":
        for i in range(current_pose[1] + 1, checkpoint[1] + 1):  # Moving right in x-axis
            path.append((current_pose[0], i))
    return path

def get_direction(current_pose, checkpoint):
    # Determine the movement direction between current_pose and checkpoint
    if current_pose[0] == checkpoint[0]:  # Same row
        return "right" if current_pose[1] < checkpoint[1] else "left"
    elif current_pose[1] == checkpoint[1]:  # Same column
        return "down" if current_pose[0] < checkpoint[0] else "up"

def get_local_path(map, current_pose, checkpoint):
    # Bypass the obstacle by finding a local path to the next checkpoint
    obstacle_direction = get_obstacle_direction(map, current_pose)
    next_checkpoints = contourn_obstacle(map, current_pose, obstacle_direction)
    return next_checkpoints

def get_obstacle_direction(map, current_pose):
    # Determine the direction of the obstacle relative to current_pose
    neighboring_cells = get_neighboring_cells(map, current_pose)
    for cell in neighboring_cells:
        if map[cell[0], cell[1]] == 1:
            return get_direction(current_pose, cell)
    return None

def get_neighboring_cells(map, current_pose):
    neighboring_cells = []
    # Iterate over all neighboring cells in a 3x3 grid
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue  # Skip the current cell
            nr, nc = current_pose[0] + dr, current_pose[1] + dc
            if 0 <= nr < map.shape[0] and 0 <= nc < map.shape[1]:  # Bounds check
                neighboring_cells.append((nr, nc))
    return neighboring_cells

def contourn_obstacle(map, current_pose, obstacle_direction):
    # Move to the side of the obstacle based on its direction
    next_checkpoints = []
    if obstacle_direction == "up":
        next_checkpoints.append((current_pose[0], current_pose[1] + 1))  # Move right
    elif obstacle_direction == "down":
        next_checkpoints.append((current_pose[0], current_pose[1] - 1))  # Move left
    elif obstacle_direction == "left":
        next_checkpoints.append((current_pose[0] + 1, current_pose[1]))  # Move down
    elif obstacle_direction == "right":
        next_checkpoints.append((current_pose[0] - 1, current_pose[1]))  # Move up
    return next_checkpoints

