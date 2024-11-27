import numpy as np
from local_navigation import run_local_navigation
from path_planning import get_global_path
import matplotlib.pyplot as plt

def visualize_map(map, current_pose, start, goal, next_checkpoints=None, title="Map Visualization"):
    """
    Visualizes the map, obstacles, robot position, goal, and trajectory.
    """
    plt.imshow(map, cmap="gray_r", origin="lower")

    if next_checkpoints is not None and len(next_checkpoints) > 1:
        next_checkpoints = np.array(next_checkpoints)
        plt.scatter(next_checkpoints[:, 1], next_checkpoints[:, 0], color="blue", label="Checkpoints")
        plt.plot(next_checkpoints[:, 1], next_checkpoints[:, 0], color="cyan", linestyle="--", label="Trajectory")
        
    if next_checkpoints is not None and len(next_checkpoints) >= 1:
        plt.plot([current_pose[1], next_checkpoints[0][1]], [current_pose[0], next_checkpoints[0][0]], color="orange", linestyle="--")

    plt.scatter(start[1], start[0], color="green", label="Start")  # Start position
    plt.scatter(goal[1], goal[0], color="red", label="Goal")  # Goal position
    plt.scatter(current_pose[1], current_pose[0], color="orange", label="Robot")  # Robot's position
    plt.legend()
    plt.title(title)
    plt.pause(1)
    plt.clf()

def test_static_obstacle():
    """
    Test Scenario 1: Static obstacles with visualization.
    """
    print("Running Test Scenario 1...")

    # Inputs
    mode = "local"
    map_shape = (240, 350)   # (rows, columns)
    map = np.zeros(map_shape)
    map[100:150, 50:80] = 1  # Static obstacle

    start = [110.5, 252.0]  # Starting position
    goal = [190, 52]  # Goal position
    current_pose = start.copy()

    # Generate the global path initially
    next_checkpoints = get_global_path(map, current_pose, goal)
    print("Initial checkpoints:", next_checkpoints)

    while next_checkpoints:
        # Visualize map
        visualize_map(map, current_pose, start, goal, next_checkpoints)

        # Perform one step of navigation
        current_pose, next_checkpoints = run_local_navigation(map, current_pose, next_checkpoints, goal, mode)

        print("The current pose is:", current_pose)

        # Check if the goal is reached
        if np.array_equal(current_pose, goal):
            print("Goal reached!")
            visualize_map(map, current_pose, start, goal, title="Goal Reached!")
            break

def test_astar_dyanmic_obstacle():
    """
    Scenario: A new obstacle appears, blocking the robot's current path.
    """
    print("Running Test Scenario: Obstacle Blocking Path...")

    # Inputs
    mode = "global"
    map_shape = (240, 350)
    map = np.zeros(map_shape)
    map[100:150, 50:80] = 1  # Static obstacle

    start = [110.5, 252.0]  # Starting position
    goal = [190, 52]  # Goal position
    current_pose = start.copy()

    # Generate the global path initially
    next_checkpoints = get_global_path(map, current_pose, goal)
    print("Initial checkpoints:", next_checkpoints)

    obstacle_added = False

    while next_checkpoints:
        
        # Add a dynamic obstacle
        if not obstacle_added and current_pose[0] > 120:
            print("Adding a dynamic obstacle...")
            map[140:160, 70:80] = 1  # New obstacle blocks part of the path
            obstacle_added = True

        visualize_map(map, current_pose, start, goal, next_checkpoints)

        # Perform one step of navigation
        current_pose, next_checkpoints = run_local_navigation(map, current_pose, next_checkpoints, goal, mode)

        print("The current pose is:", current_pose)

        # Check if the goal is reached
        if np.array_equal(current_pose, goal):
            print("Goal reached!")
            visualize_map(map, current_pose, start, goal, title="Goal Reached!")
            break

def test_contourning_dynamic_obstacle():
    """
    Scenario: A new obstacle appears, blocking the robot's current path.
    """
    print("Running Test Scenario: Obstacle Blocking Path...")

    # Inputs
    mode = "local"
    map_shape = (240, 350)
    map = np.zeros(map_shape)
    map[100:150, 50:80] = 1  # Static obstacle

    start = [110.5, 252.0]  # Starting position
    goal = [190, 52]  # Goal position
    current_pose = start.copy()

    # Generate the global path initially
    next_checkpoints = get_global_path(map, current_pose, goal)
    print("Initial checkpoints:", next_checkpoints)

    obstacle_added = False

    while next_checkpoints:
        
        # Add a dynamic obstacle
        if not obstacle_added and current_pose[0] > 120:
            print("Adding a dynamic obstacle...")
            map[140:160, 70:80] = 1  # New obstacle blocks part of the path
            obstacle_added = True

        visualize_map(map, current_pose, start, goal, next_checkpoints)

        # Perform one step of navigation
        current_pose, next_checkpoints = run_local_navigation(map, current_pose, next_checkpoints, goal, mode)

        print("The current pose is:", current_pose)

        # Check if the goal is reached
        if np.array_equal(current_pose, goal):
            print("Goal reached!")
            visualize_map(map, current_pose, start, goal, title="Goal Reached!")
            break

if __name__ == "__main__":
    print("Starting Navigation Tests...")

    # try:
    #     test_static_obstacle()
    # except KeyboardInterrupt:
    #     print("Test Scenario 1 interrupted by user.")

    # try:
    #     test_astar_dyanmic_obstacle()
    # except KeyboardInterrupt:
    #     print("Test Scenario 2 interrupted by user.")

    try:
        test_contourning_dynamic_obstacle()
    except KeyboardInterrupt:
        print("Test Scenario 3 interrupted by user.")