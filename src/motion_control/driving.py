from tdmclient import ClientAsync, aw
import time
import numpy as np
from simple_pid import PID

class Driving:
    def __init__(self, px2mm=3):
        self.wheel_radius = 20  # [mm]
        self.max_wheel_speed = 0.7  # [revolution per second at 500]
        self.r_speed = 66 #[mm/s]
        self.l_speed = 66 #[mm/s]
        self.px2mm = px2mm
        self.time = 0 
        self.client = ClientAsync()
        self.client.process_waiting_messages()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.unlock())
        print("Thymio connected:", self.node)
        aw(self.node.lock())
        self.prox_horizontal = [0, 0, 0, 0, 0]  # Initial horizontal proximity sensor values
        self.prox_ground = [1000, 1000]
        self.sensor_scale = 400  # Scale of the proximity sensors
        self.pid = PID(60, 0, 0, setpoint=0)
        aw(self.initialize_node_listeners())

    async def initialize_node_listeners(self):
        # Watch for variable changes
        await self.node.watch(variables=True)

        # Add listener for proximity sensor changes
        self.node.add_variables_changed_listener(self.on_variables_changed) 

    def on_variables_changed(self, node, variables):
        """
        Callback function to handle variable updates.
        Updates proximity sensor values when 'prox.horizontal' changes.
        This was inspired by this example: https://pypi.org/project/tdmclient/#:~:text=To%20read%20variables,typing%20control%2DC.
        """
        try:
            if "prox.horizontal" in variables:
                self.prox_horizontal = list(variables["prox.horizontal"])
            if "prox.ground.reflected" in variables:
                self.prox_ground = list(variables["prox.ground.reflected"])
        except KeyError:
            pass
 
    async def get_prox_horizontal(self):
        """
        Returns the latest front horizontal proximity sensor values.

        Outputs:
        - prox_horizontal: List of the front 5 horizontal proximity sensor values
        """
        await self.client.sleep(0.1)  # Wait for the latest values to be updated
        # print(f"Returning proximity sensor values: {self.prox}")
        self.prox_horizontal = self.prox_horizontal[:5]
        self.prox_horizontal = [x // self.sensor_scale for x in self.prox_horizontal]
        return self.prox_horizontal
    
    async def get_prox_ground(self):
        """
        Returns the latest ground proximity sensor values.

        Outputs:
        - prox_ground: List of the ground proximity sensor values
        """
        await self.client.sleep(0.1)  # Wait for the latest values to be updated
        return self.prox_ground

    def __del__(self):
        aw(self.node.unlock())
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
        # timer = threading.Timer(duration, self.stop)
        # timer.start()

        # Stop the motors
        v = {
            "motor.left.target": [0],
            "motor.right.target": [0],
        }
        aw(self.node.set_variables(v))

    def get_l_speeds (self):
        return self.l_speed 

    def get_r_speeds (self):
        return self.r_speed 

    def get_time (self):
        return self.time

    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set the motor speeds of the left and right motors.

        :param left_speed: Speed of the left motor
        :param right_speed: Speed of the right motor
        """

        # print(f"Setting motor speeds: Left = {left_speed}, Right = {right_speed}")
        v = {
            "motor.left.target": [left_speed],
            "motor.right.target": [right_speed],
        }
        # TODO: define mapping target speed to speed in mm/s as global variable and justify it
        self.r_speed = right_speed / 3 
        self.l_speed = left_speed  / 3
        aw(self.node.set_variables(v))

    def turn(self, angle):
        """ Turns a given angle in radians

        Args:
            angle (float): Angle to turn in radians
        """
        speed = 200
        scaling_factor = 0.73 # empirical value

        if angle >= 0:
            duration = abs((angle)*scaling_factor)  
            self.execute_command(-speed, speed, duration)
            # print(f"Turning {angle} rad")
        

        elif angle < 0:
            duration = abs((angle)*scaling_factor)
            self.execute_command(speed, -speed, duration)
            # print(f"Turning {angle} rad")

    # TODO: Function not used, remove (or maybe use for local avoidance)?
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

    def stop(self):
        """
        Stop the robot.
        """
        self.set_motor_speeds(0, 0)

    def px_to_mm(self, val):
        return self.px2mm*val

    def move_to_checkpoint(self, robot_pose: tuple[np.ndarray, float], checkpoint: np.ndarray):
        """
        Function to move the robot to a given point in the map. P(ID) control on the
        angle is used to steer the robot towards the checkpoint.

        Args:
            robot_pose (tuple): Current robot pose as a tuple of (position, orientation)
            checkpoint (np.ndarray): Coordinates of the checkpoint [x, y]
        """
        if np.isnan(robot_pose[0]).any():
            return

        dx = checkpoint[0] - robot_pose[0][0]
        dy = robot_pose[0][1] - checkpoint[1]
        
        dir = np.arctan2(dy, dx)

        # angle between robot's orientation and direction to the checkpoint
        angle = dir - robot_pose[1]

        # Normalize angle to [-pi, pi] (i.e. alwas turn into the shortest direction)
        if (abs(angle) > np.pi):
            angle = angle - 2*np.pi*np.sign(angle)

        # If the angle is too large, turn in place, here we don't do local avoidance
        # as we turn on the spot
        if abs(angle) > np.deg2rad(25):
            self.turn(angle)
        
        # Otherwise, move forward with a P(ID) control to steer towards the checkpoint
        else:
            gain = self.pid(angle)
            l_speed = 100 + gain
            r_speed = 100 - gain

            self.set_motor_speeds(int(l_speed), int(r_speed))

    def get_motor_speeds(self):
        """
        Returns the current speeds of the left and right motors.
        
        Outputs:
        - left_speed: Speed of the left motor
        - right_speed: Speed of the right motor
        """
        aw(self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"}))
        # Fetch motor speed values from Thymio's variables
        left_speed = self.node["motor.left.speed"]
        right_speed = self.node["motor.right.speed"]

        # Convert to mm/s using the calibration factor
        return left_speed/3, right_speed/3
    # except KeyError as e:
        # print("Error: Unable to fetch motor speeds. Are the motor variables available?{e}")
        # return None, None


    