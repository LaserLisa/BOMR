from tdmclient import ClientAsync, aw
import time

wheel_radius = 20 #[mm]
max_wheel_speed = 0.7 #[revolution per second at 500] 

def execute_command(node, left_speed, right_speed, duration):
    """Set motor speeds, wait for the duration, and stop the motors."""
    print(f"Executing command: Left = {left_speed}, Right = {right_speed}, Duration = {duration}s")
    
    # Set motor speeds
    v = {
        "motor.left.target": [left_speed],
        "motor.right.target": [right_speed],
    }
    aw(node.set_variables(v))

    # Wait for the specified duration
    time.sleep(duration)

    # Stop the motors
    v = {
        "motor.left.target": [0],
        "motor.right.target": [0],
    }
    aw(node.set_variables(v))
    print("Movement completed.\n")

def turn(node, degrees):
    speed = 200

    degrees = degrees%360 #normalization
    
    if degrees >= 180:
        duration = 1.5 #TODO
        execute_command(node, -speed, speed, duration)

    elif degrees < 180:
        duration = 1.5 #TODO
        execute_command(node, speed, -speed, duration)

def move(node, duration):
    speed = 200
    #some scaling between pixels and mm?
    execute_command(node, speed, speed, duration)




def main():
    client = ClientAsync()
    client.process_waiting_messages()
    node = aw(client.wait_for_node())
    aw(node.unlock())
    print("Thymio connected:", node)
    aw(node.lock())

    try:
        while True:
            # Get user input for the command
            command = input("Enter command ('turn degrees' or 'move duration') or 'exit' to quit: ").strip()
            if command.lower() == "exit":
                print("Exiting program.")
                break

            # Parse the command
            try:
                if command.startswith("turn"):
                    _, degrees = command.split()
                    degrees = int(degrees)
                    turn(node, degrees)
                elif command.startswith("move"):
                    _, duration = command.split()
                    duration = int(duration)
                    move(node, duration)
                else:
                    print("Invalid command. Use 'turn degrees' or 'move duration'.")
            except ValueError:
                print("Invalid input. Please ensure the correct format for 'turn degrees' or 'move duration'.")
    finally:
        aw(node.lock())
        client.close()

if __name__ == "__main__":
    main()