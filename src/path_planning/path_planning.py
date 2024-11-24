# Input : matrice 80 x 100 where obstacles are represented by 1 and free space is represented by 0
#         start and goal coordinates (l, c) where l is the line and c is the column

# Output: optimal path from start to goal coordinates

import numpy as np
import heapq
import matplotlib.pyplot as plt

def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    def heuristic(a, b):
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

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
                move_cost = 1.05 if dr != 0 and dc != 0 else 1
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

def inflate_obstacles(grid, robot_radius):
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

def inflate_borders(grid, robot_width):
    rows, cols = grid.shape
    inflated_grid = grid.copy()
    for r in range(rows):
        for c in range(cols):
            if r < robot_width or r >= rows - robot_width or c < robot_width or c >= cols - robot_width:
                inflated_grid[r, c] = 1
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
grid = np.zeros((rows, cols))

# Add obstacles to the grid
grid[20:60, 30] = 1
grid[40, 50:90] = 1

# Define start and goal points
start = (6, 6)
goal = (73, 93)

# Define robot size
robot_width = 12
robot_radius = robot_width // 2

# Inflate the obstacles
inflated_grid = inflate_obstacles(grid, robot_radius)
inflated_grid = inflate_borders(inflated_grid, robot_radius - 1)

# Run A* algorithm on the inflated grid
path = a_star(inflated_grid, start, goal)
checkpoints = get_checkpoints(path)

# Visualize the result
plot_grid_with_inflation_and_checkpoints(grid, inflated_grid, checkpoints, path=path, start=start, goal=goal)