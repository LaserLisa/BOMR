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

def test_scenario_1():
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



def test_scenario_2():
    """
    Test Scenario 2: Dynamic obstacles with visualization.
    """
    print("Running Test Scenario 2...")

    # Inputs
    mode = "local"
    map_shape = (240, 350)   # (rows, columns)
    map = np.zeros(map_shape)
    map[100:150, 50:80] = 1  # Initial static obstacle

    start = [110.5, 252.0]  # Starting position
    goal = [190, 52]  # Goal position
    current_pose = start.copy()

    # Generate the global path initially
    next_checkpoints = get_global_path(map, current_pose, goal)
    print("Initial checkpoints:", next_checkpoints)

    obstacle_added = False  # Track if dynamic obstacle has been added

    while True:
        visualize_map(map, current_pose, start, goal, next_checkpoints)

        # Simulate addition of a dynamic obstacle after reaching the first checkpoint
        if not obstacle_added and current_pose[0] > 120:
            print("Adding a dynamic obstacle...")
            map[140:160, 70:90] = 1  # Add a new obstacle
            obstacle_added = True

        # Run navigation for the next step
        run_local_navigation(map, current_pose, next_checkpoints, goal, mode=mode)
        print("The current pose is:", current_pose)

        # Simulate robot moving to the next position
        if next_checkpoints:
            current_pose[:] = list(next_checkpoints.pop(0))  # Move to the next checkpoint

        # Check if the goal is reached
        if np.array_equal(current_pose, goal):
            print("Goal reached!")
            visualize_map(map, current_pose, start, goal, title="Goal Reached!")
            break
def test_scenario_3():
    """
    Test Scenario 3: Multiple dynamic obstacles with visualization.
    """
    print("Running Test Scenario 3...")

    # Inputs
    mode = "local"
    map_shape = (240, 350)   # (rows, columns)
    map = np.zeros(map_shape)
    map[100:150, 50:80] = 1  # Initial static obstacle

    start = [110.5, 252.0]  # Starting position
    goal = [190, 52]  # Goal position
    current_pose = start.copy()

    # Generate the global path initially
    next_checkpoints = get_global_path(map, current_pose, goal)
    print("Initial checkpoints:", next_checkpoints)

    obstacle_added_1 = False
    obstacle_added_2 = False

    while True:
        visualize_map(map, current_pose, start, goal, next_checkpoints)

        # Add first dynamic obstacle
        if not obstacle_added_1 and current_pose[0] > 120:
            print("Adding first dynamic obstacle...")
            map[140:160, 70:90] = 1  # Add a new obstacle
            obstacle_added_1 = True

        # Add second dynamic obstacle
        if not obstacle_added_2 and current_pose[0] > 160:
            print("Adding second dynamic obstacle...")
            map[170:190, 100:120] = 1  # Add another new obstacle
            obstacle_added_2 = True

        # Run navigation for the next step
        run_local_navigation(map, current_pose, next_checkpoints, goal, mode=mode)
        print("The current pose is:", current_pose)

        # Simulate robot moving to the next position
        if next_checkpoints:
            current_pose[:] = list(next_checkpoints.pop(0))  # Move to the next checkpoint

        # Check if the goal is reached
        if np.array_equal(current_pose, goal):
            print("Goal reached!")
            visualize_map(map, current_pose, start, goal, title="Goal Reached!")
            break



if __name__ == "__main__":
    print("Starting Navigation Tests...")

    try:
        test_scenario_1()
    except KeyboardInterrupt:
        print("Test Scenario 1 interrupted by user.")

    # try:
    #     test_scenario_2()
    # except KeyboardInterrupt:
    #     print("Test Scenario 2 interrupted by user.")

    # try:
    #     test_scenario_3()
    # except KeyboardInterrupt:
    #     print("Test Scenario 3 interrupted by user.")