# Input : matrice 80 x 100 where obstacles are represented by 1 and free space is represented by 0
#         start and goal coordinates (l, c) where l is the line and c is the column

# Output: optimal path from start to goal coordinates

import numpy as np
import heapq
import matplotlib.pyplot as plt

def a_star(grid, start, goal):
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
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(12, 10))

    # Create a visual grid with color codes
    display_grid = np.zeros_like(grid, dtype=float)
    display_grid[inflated_grid == 0] = 0.5  # Default free space as light grey
    display_grid[grid == 1] = 0  # Original obstacles as black
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
            ax.scatter(c, r, c="orange", s=50)

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

def get_checkpoints(map, start, goal, px2mm):
    # Define robot size
    robot_width = 12
    robot_radius = robot_width / 2
    
    # transform cm to px
    robot_radius = int(robot_radius*10/px2mm)

    # Inflate the obstacles
    inflated_grid = inflate_obstacles(map, robot_radius)
    inflated_grid = inflate_borders(inflated_grid, robot_radius - 1)

    # Run A* algorithm on the inflated grid
    path = a_star(inflated_grid, start, goal)

    if not path:
        return []
    checkpoints = [[path[0][1], path[0][0]]]
    for i in range(1, len(path) - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        if (x2 - x0) * (y1 - y0) != (x1 - x0) * (y2 - y0):
            checkpoints.append([path[i][1], path[i][0]])

    plot_grid_with_inflation_and_checkpoints(map, inflated_grid, checkpoints, path=path, start=start, goal=goal)
    return checkpoints


# map_shape = (240, 350)
# grid = np.zeros(map_shape)
# grid[100:150, 50:80] = 1  # Example obstacle

# start = np.array([252.0, 110.5])  # Note: x and y will be swapped
# goal = np.array([52, 190])

# robot_width = 12
# robot_radius = robot_width // 2

# inflated_grid = inflate_obstacles(grid, robot_radius)
# inflated_grid = inflate_borders(inflated_grid, robot_radius - 1)

# path = a_star(inflated_grid, start, goal)
# checkpoints = get_checkpoints(path)

# plot_grid_with_inflation_and_checkpoints(grid, inflated_grid, checkpoints, path=path, start=start, goal=goal)
