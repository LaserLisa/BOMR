# Input : matrice 80 x 100 where obstacles are represented by 0 and free space is represented by 1
#         start and goal coordinates (l, c) where l is the line and c is the column

# Output: optimal path from start to goal coordinates

import numpy as np
import heapq
import matplotlib.pyplot as plt

def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    # 8 possible movements including diagonals
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),  # Up, Down, Left, Right
        (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonals
    ]

    def heuristic(a, b):
        # Use Euclidean distance for diagonal movement
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

    open_set = []                             # A priority queue to store nodes to explore
    heapq.heappush(open_set, (0, start))      # (f, (row, col))
    came_from = {}                            # Parent of each node
    g_score = {start: 0}                      # Cost of the cheapest path from start to a node
    f_score = {start: heuristic(start, goal)} # g_score + heuristic

    while open_set: # While there are nodes to explore
        _, current = heapq.heappop(open_set) # Get node with the lowest f_score

        if current == goal: # If we reached the goal, reconstruct the path by tracing back through parents
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 1:
                # Adjust cost for diagonal movement
                move_cost = 1.1 if dr != 0 and dc != 0 else 1 # if diagonal movement, slightly discourage it as it's less efficient
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]: # If the neighbor is not visited yet or we found a cheaper path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # No path found

def inflate_obstacles(grid, robot_radius):
    """Inflates obstacles in the grid to account for the robot's width."""
    rows, cols = grid.shape
    inflated_grid = grid.copy()
    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 0:  # If this cell is an obstacle
                # Mark cells within the robot's radius as obstacles
                for dr in range(-robot_radius, robot_radius + 1):
                    for dc in range(-robot_radius, robot_radius + 1):
                        nr, nc = r + dr, c + dc
                        # Ensure within bounds
                        if 0 <= nr < rows and 0 <= nc < cols:
                            inflated_grid[nr, nc] = 0
    return inflated_grid

def plot_grid_with_inflation_and_checkpoints(grid, inflated_grid, checkpoints, path=None, start=None, goal=None):
    """Plots the grid with inflated cells in light gray."""
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Create a visual grid: light gray for inflated, black for actual obstacles
    display_grid = np.zeros_like(grid, dtype=float)
    display_grid[inflated_grid == 0] = 0.5  # Inflated obstacles (light gray)
    display_grid[grid == 0] = 0  # Original obstacles (black)
    display_grid[inflated_grid == 1] = 1  # Free space (white)
    
    # Plot grid
    ax.imshow(display_grid, cmap="Greys", origin="upper")
    
    # Plot start and goal
    if start:
        ax.scatter(start[1], start[0], c="green", s=100, label="Start")
    if goal:
        ax.scatter(goal[1], goal[0], c="red", s=100, label="Goal")
    
    # Plot path
    if path:
        path_coords = np.array(path)
        ax.plot(path_coords[:, 1], path_coords[:, 0], c="blue", label="Path", linewidth=2)
        ax.set_title(f"Path Length: {len(path)}", fontsize=14)

    # Plot checkpoints
    if checkpoints:
        for r, c in checkpoints:
            ax.scatter(c, r, c="orange", s=50)
    
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="black", linestyle="-", linewidth=0.5)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax.legend()
    plt.show()

def get_checkpoints(path):
    """Returns the checkpoints from the path."""
    if not path:
        return []
    checkpoints = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        if (x2 - x0) * (y1 - y0) != (x1 - x0) * (y2 - y0):
            checkpoints.append(path[i])
    return checkpoints

# Define an 80x100 grid
rows, cols = 80, 100
grid = np.ones((rows, cols))

# Add obstacles to the grid
grid[20:60, 30] = 0
grid[40, 50:90] = 0

# Define start and goal points
start = (0, 0)
goal = (79, 99)

# Define robot size
robot_width = 12 # The robot has a width of 11cm
robot_radius = robot_width // 2

# Inflate the obstacles
inflated_grid = inflate_obstacles(grid, robot_radius)

# Run A* algorithm on the inflated grid
path = a_star(inflated_grid, start, goal)
checkpoints = get_checkpoints(path)

# Visualize the result
plot_grid_with_inflation_and_checkpoints(grid, inflated_grid, checkpoints, path=path, start=start, goal=goal)