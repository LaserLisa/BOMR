from tdmclient import ClientAsync, aw
import time
import math

class Driving:
    def __init__(self):
        self.wheel_radius = 20  # [mm]
        self.max_wheel_speed = 0.7  # [revolution per second at 500]
        self.r_speed = 66 #[mm/s]
        self.l_speed = 66 #[mm/s]
        self.time = 0 
        self.client = ClientAsync()
        self.client.process_waiting_messages()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.unlock())
        print("Thymio connected:", self.node)
        aw(self.node.lock())
        self.x = 0
        self.y = 0
        self.dir = 0

    def __del__(self):
        aw(self.node.lock())
        aw(self.node.stop())
        self.client.close()
        print("Thymio connection closed.")

    def execute_command(self, left_speed, right_speed, duration):
        """Set motor speeds, wait for the duration, and stop the motors."""
        #print(f"Executing command: Left = {left_speed}, Right = {right_speed}, Duration = {duration}s")

        # Set motor speeds
        v = {
            "motor.left.target": [left_speed],
            "motor.right.target": [right_speed],
        }
        self.r_speed = right_speed / 3 
        self.l_speed = left_speed  / 3
        self.time = duration 
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

    def get_l_speeds (self):
        return self.l_speed 

    def get_r_speeds (self):
        return self.r_speed 

    def get_time (self):
        return self.time

    
    def turn(self, angle):
        speed = 200
        scaling_factor = 0.73 # empirical value

        if angle >= 0:
            duration = abs((angle)*scaling_factor)  
            self.execute_command(-speed, speed, duration)
            print(f"Turning {angle} rad")
        

        elif angle < 0:
            duration = abs((angle)*scaling_factor)
            self.execute_command(speed, -speed, duration)
            print(f"Turning {angle} rad")
    def move(self, distance):
        """
        Move the robot forward for a given distance in millimeters.

        :param distance: Distance to move in mm
        """
        speed = 200  # Motor speed
        scaling_factor = 0.014  # empirical

        # Calculate duration using the scaling factor
        duration = distance * scaling_factor

        print(f"Moving {distance} mm")
        self.execute_command(speed, speed, duration)

    def px_to_mm(self, val):
        return 3*val

    def move_to_checkpoint(self, pos_x, pos_y, pos_angle, check_x, check_y):
        #x, y coordinates are in milimeters, pixel to mm mapping TBD more precisely

        print(f"move between {pos_x}, {pos_y} and {check_x}, {check_y}")
    
        dx = check_x - pos_x
        dy = check_y - pos_y
        
        dir = math.atan2(dy, dx)

        self.turn(dir - pos_angle)
        self.move(self.px_to_mm(math.sqrt(pow(dx, 2)+pow(dy, 2))))

        self.dir = dir
        self.x = check_x
        self.y = check_y


    def get_motor_speeds(self):
        """
        Returns the current speeds of the left and right motors.
        
        Outputs:
        - left_speed: Speed of the left motor
        - right_speed: Speed of the right motor
        """
    # try:
        # Fetch motor speed values from Thymio's variables
        left_speed = self.node["motor.left.speed"]
        right_speed = self.node["motor.right.speed"]

        # Convert to mm/s using the calibration factor
        left_speed = 0.4 * left_speed
        right_speed = 0.4 * right_speed
        return left_speed, right_speed
    # except KeyError as e:
        # print("Error: Unable to fetch motor speeds. Are the motor variables available?{e}")
        # return None, None

