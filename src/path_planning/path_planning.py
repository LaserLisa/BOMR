import numpy as np
import heapq
import matplotlib.pyplot as plt

def a_star(grid, start, goal):
    """
    Find the optimal path from start to goal on a grid using the A* algorithm.

    Args:
        grid (numpy.ndarray): Binary grid where 0 is free space and 1 is an obstacle.
        start (tuple): Start coordinates as (row, col).
        goal (tuple): Goal coordinates as (row, col).

    Returns:
        list: List of (row, col) tuples representing the path from start to goal, or an empty list if the path is blocked.

    Notes:
        - This code is inspired from this tutorial: https://llego.dev/posts/implementing-the-a-search-algorithm-python/
    """
    rows, cols = grid.shape
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    def heuristic(a, b):
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

    # Swap x and y coordinates for correct indexing
    start = tuple(map(int, np.round(start[::-1])))  # Convert to (row, col)
    goal = tuple(map(int, np.round(goal[::-1])))    # Convert to (row, col)

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                move_cost = 1.5 if dr != 0 and dc != 0 else 1
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

def inflate_obstacles(grid, robot_radius):
    """ 
    Inflate the obstacles in the grid by the robot radius.

    Args:
        grid (numpy.ndarray): Binary grid where 0 is free space and 1 is an obstacle.
        robot_radius (int): Radius of the robot in pixels.

    Returns:
        numpy.ndarray: Inflated grid where obstacles are marked as 1.
    """
    rows, cols = grid.shape
    inflated_grid = grid.copy()
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                for dr in range(-robot_radius, robot_radius + 1):
                    for dc in range(-robot_radius, robot_radius + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            inflated_grid[nr, nc] = 1
    return inflated_grid

def inflate_borders(grid, robot_radius):
    """
    Inflate the borders of the grid by the robot width.

    Args:
        grid (numpy.ndarray): Binary grid where 0 is free space and 1 is an obstacle.
        robot_radius (int): Width of the robot in pixels.

    Returns:
        numpy.ndarray: Inflated grid where borders are marked as 1.
    """
    (rows, cols) = grid.shape
    inflated_grid = grid.copy()
    for r in range(rows):
        for c in range(cols):
            if r < robot_radius or r >= rows - robot_radius or c < robot_radius or c >= cols - robot_radius:
                inflated_grid[r, c] = 1
    return inflated_grid

def plot_grid_with_inflation_and_checkpoints(grid, inflated_grid, checkpoints, path=None, start=None, goal=None):
    """
    Plot the grid with obstacles, inflated obstacles, checkpoints, and path.

    Args:
        grid (numpy.ndarray): Binary grid where 0 is free space and 1 is an obstacle.
        inflated_grid (numpy.ndarray): Inflated grid where obstacles are marked as 1.
        checkpoints (list): List of checkpoints as [x, y] pairs.
        path (list): List of [x, y] coordinates representing the path.
        start (tuple): Start coordinates as (row, col).
        goal (tuple): Goal coordinates as (row, col).
    """
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(12, 10))

    # Create a visual grid with color codes
    display_grid = np.zeros_like(grid, dtype=float)
    display_grid[inflated_grid == 0] = 0.5                  # Default free space as light grey
    display_grid[grid == 1] = 0                             # Original obstacles as black
    display_grid[(inflated_grid == 1) & (grid == 0)] = 0.2  # Contours as dark grey

    # Plot the grid
    ax.imshow(display_grid, cmap="Greys", origin="upper")

    # Swap coordinates for start and goal to match plotting
    if start is not None:
        start = start[::-1]
        ax.scatter(start[1], start[0], c="green", s=100, label="Start")
    if goal is not None:
        goal = goal[::-1]
        ax.scatter(goal[1], goal[0], c="red", s=100, label="Goal")

    # Plot path
    if path:
        path_coords = np.array(path)
        ax.plot(path_coords[:, 1], path_coords[:, 0], c="blue", label="Path", linewidth=2)
        ax.set_title(f"Path Length: {len(path)}", fontsize=14)

    # Plot checkpoints
    if checkpoints:
        for r, c in checkpoints:
            ax.scatter(r, c, c="orange", s=50)

    # Add row and column labels at intervals of 20
    for x in range(0, cols, 20):
        ax.text(x, rows - 1 + 2, f"{x}", color="black", fontsize=10, ha="center")
    for y in range(0, rows, 20):
        ax.text(-2, y, f"{y}", color="black", fontsize=10, va="center")

    # Remove ticks
    ax.set_xticks([])
    ax.set_yticks([])
    ax.tick_params(left=False, bottom=False, labelleft=True, labelbottom=True)

    ax.legend()
    plt.show()

def bresenham_line(x1, y1, x2, y2):
    """
    Generate all the points on a straight line between two points using Bresenham's algorithm.
    
    Args:
        x1, y1 (int): Coordinates of the start point.
        x2, y2 (int): Coordinates of the end point.
    
    Returns:
        list: List of (x, y) tuples representing the line.
    """
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points

def is_visible(point1, point2, inflated_grid):
    """
    Check if there is visibility between two points, ensuring no pixels along the line
    belong to inflated obstacles.
    
    Args:
        point1 (list or tuple): Coordinates of the first point [x, y].
        point2 (list or tuple): Coordinates of the second point [x, y].
        inflated_grid (numpy.ndarray): Inflated grid where obstacles are marked as 1.
    
    Returns:
        bool: True if the line is clear, False otherwise.
    """
    x1, y1 = map(int, point1)
    x2, y2 = map(int, point2)

    # Get all points on the line
    line_points = bresenham_line(x1, y1, x2, y2)

    # Check if any point on the line intersects an obstacle
    for x, y in line_points:
        if 0 <= y < inflated_grid.shape[0] and 0 <= x < inflated_grid.shape[1]:
            if inflated_grid[y, x] == 1:
                return False
    return True

def visibility_graph_simple(checkpoints, inflated_grid):
    """
    Removes checkpoints that are redundant because their two neighbors can directly see each other.
    Already skipped checkpoints are not reconsidered.

    Args:
        checkpoints (list): List of checkpoints as [x, y] pairs.
        inflated_grid (numpy.ndarray): Inflated grid where obstacles are marked as 1.

    Returns:
        list: Updated list of checkpoints with redundant ones removed.
    """
    # Work with a copy of the checkpoints list to avoid modifying the original list
    filtered_checkpoints = checkpoints[:]
    i = 1  # Start with the second checkpoint

    while i < len(filtered_checkpoints) - 1:
        prev = filtered_checkpoints[i - 1]
        current = filtered_checkpoints[i]
        next_ = filtered_checkpoints[i + 1]

        # Check if the current checkpoint can be skipped
        if is_visible(prev, next_, inflated_grid):
            # Remove the current checkpoint
            filtered_checkpoints.pop(i)
        else:
            # Move to the next checkpoint
            i += 1

    return filtered_checkpoints

def get_checkpoints(map, start, goal, px2mm, plot=False):
    """
    Compute the checkpoints for the robot to follow from start to goal.

    Args:
        map (numpy.ndarray): Binary grid where 0 is free space and 1 is an obstacle.
        start (tuple): Start coordinates as (row, col).
        goal (tuple): Goal coordinates as (row, col).
        px2mm (float): Conversion factor from pixels to millimeters.
        plot (bool): Whether to plot the grid with checkpoints.

    Returns:
        list: List of checkpoints as [x, y] pairs.
    """
    # Define robot size
    robot_radius = 8
    
    # transform cm to px
    robot_radius = int(robot_radius*10/px2mm)
    print(f"robot_radius: {robot_radius}")

    # Inflate the obstacles
    inflated_grid = inflate_obstacles(map, robot_radius)
    inflated_grid = inflate_borders(inflated_grid, robot_radius - 1)

    smaller_inflated_grid = inflate_obstacles(map, robot_radius-1)
    smaller_inflated_grid = inflate_borders(smaller_inflated_grid, robot_radius-1)
    
    # Run A* algorithm on the inflated grid
    path = a_star(inflated_grid, start, goal)

    if not path:
        return []
    checkpoints = [[path[0][1], path[0][0]]]

    # Only keep checkpoints that represent a change in direction
    for i in range(1, len(path) - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        if (x2 - x0) * (y1 - y0) != (x1 - x0) * (y2 - y0):
            checkpoints.append([path[i][1], path[i][0]])

    checkpoints = visibility_graph_simple(checkpoints, smaller_inflated_grid)

    if plot:
        plot_grid_with_inflation_and_checkpoints(map, smaller_inflated_grid, checkpoints, path=path, start=start, goal=goal)
    return checkpoints

"""
Text to add in report:
We receive from the vision system a binary grid representing the environment, where obstacles are marked as 1 and free space as 0.
Before finding the optimal path between the start and goal coordinates, we first need to determine which grid cells we can traverse. 
Since, the robot's position is determined by it's center, we inflate the obstacles by the robot's radius to ensure that it does not collide with any obstacles.

Next, we use the A* algorithm to compute the optimal path between the start and goal coordinates. 
The algorithm returns a series of grid cells that the robot should traverse. 
However, not all of these grid cells are necessary for defining the path; many are redundant as they lie along straight segments of the path without any change in direction.

To simplify the path, we filter out the redundant points and retain only the checkpoints that represent a change in direction. 
For each point, we compute the vectors formed with its neighboring points and check for collinearity using the cross-product. 
Points that do not result in a direction change are excluded, leaving only the essential checkpoints that guide the robot through turns and direction changes. 

"""