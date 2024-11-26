from tdmclient import ClientAsync, aw
import time
import math

class Driving:
    def __init__(self):
        self.wheel_radius = 20  # [mm]
        self.max_wheel_speed = 0.7  # [revolution per second at 500]
        self.client = ClientAsync()
        self.client.process_waiting_messages()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.unlock())
        print("Thymio connected:", self.node)
        aw(self.node.lock())

    def __del__(self):
        aw(self.node.lock())
        self.client.close()
        print("Thymio connection closed.")

    def execute_command(self, left_speed, right_speed, duration):
        """Set motor speeds, wait for the duration, and stop the motors."""
        print(f"Executing command: Left = {left_speed}, Right = {right_speed}, Duration = {duration}s")

        # Set motor speeds
        v = {
            "motor.left.target": [left_speed],
            "motor.right.target": [right_speed],
        }
        aw(self.node.set_variables(v))

        # Wait for the specified duration
        time.sleep(duration)

        # Stop the motors
        v = {
            "motor.left.target": [0],
            "motor.right.target": [0],
        }
        aw(self.node.set_variables(v))
        print("Movement completed.\n")

    def turn(self, degrees):
        speed = 200
        degrees = degrees % 360  # normalization

        if degrees >= 180:
            duration = 1.5  # TODO: Adjust for accurate turning
            self.execute_command(speed, -speed, duration)

        elif degrees < 180:
            duration = 1.5  # TODO: Adjust for accurate turning
            self.execute_command(-speed, speed, duration)

    def move(self, duration):
        speed = 200
        # Some scaling between pixels and mm?
        self.execute_command(speed, speed, duration)

    def px_to_mm(self, val):
        return 3*val

    def move_to_checkpoint(self, pos_x, pos_y, pos_angle, check_x, check_y):
        #I assume pos_angle to be 0 when facing North and go towards 360 counter-clockwise!
        
        #calculate rotation & distance
        dx = check_x - pos_x
        dy = check_y - pos_y
        
        if(dx == 0 and dy >= 0):
            dir = 0
        elif(dx == 0 and dy < 0):
            dir = 180
        elif(dx > 0):
            dir = math.degrees(math.atan(dy/dx)) + 270
        elif(dx < 0):
            dir = math.degrees(math.atan(dy/dx)) + 90

        self.turn(dir - pos_angle)
        self.move(self.px_to_mm(math.sqrt(pow(dx, 2)+pow(dy, 2))))
