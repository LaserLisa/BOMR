from tdmclient import ClientAsync, aw
import time
import math
import numpy as np
import asyncio

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
        self.prox = [0, 0, 0, 0, 0, 0, 0]  # Initial horizontal proximity sensor values
        self.sensor_scale = 200  # Scale of the proximity sensors
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
        """
        try:
            if "prox.horizontal" in variables:
                self.prox = list(variables["prox.horizontal"])
        except KeyError:
            pass
 
    async def get_prox_horizontal(self):
        """
        Returns the latest front horizontal proximity sensor values.

        Outputs:
        - prox_horizontal: List of the front 5 horizontal proximity sensor values
        """
        await self.client.sleep(0.1)  # Wait for the latest values to be updated
        print(f"Returning proximity sensor values: {self.prox}")
        self.prox = [self.prox[0] // self.sensor_scale, self.prox[1] // self.sensor_scale, self.prox[2] // self.sensor_scale, self.prox[3] // self.sensor_scale, self.prox[4] // self.sensor_scale]
        return self.prox

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

    def stop(self):
        """
        Stop the robot.
        """
        print("Stopping the robot")
        self.execute_command(0, 0, 0)

    def px_to_mm(self, val):
        return 3*val

    def move_to_checkpoint(self, robot_pose: tuple[np.ndarray, float], checkpoint: np.ndarray):
        #x, y coordinates are in px, pixel to mm mapping TBD more precisely
        if np.isnan(robot_pose[0]).any():
            return
        dx = checkpoint[0] - robot_pose[0][0]
        dy = robot_pose[0][1] - checkpoint[1] 

        print(f"dx = {dx}")
        print(f"dy = {dy}")

        print(f"robot_pose[1] = {robot_pose[1]}")
        dir = np.arctan2(dy, dx)
        print(f"dir = {dir}")

        angle = dir - robot_pose[1]
        print(f"angle = {angle}")
        # print(f"angle error: {np.degrees(angle)}")
        # print(f"dir error: {np.degrees(dir)}", end="")
        # angle = angle if abs(angle) > np.deg2rad(20) else 0
        # P = 60
        # l_speed = 100 - P * angle
        # r_speed = 100 + P * angle
        distance = np.linalg.norm([dx, dy])
        print(f"distance px = {distance}")
        print(f"distance mm = {self.px_to_mm(distance)}")

        print(f"calling turn(angle) and move(self.px_to_mm(distance))")
        self.turn(angle)
        self.move(self.px_to_mm(distance))
        # print(f"sending speeds: {l_speed}, {r_speed}")
        # v = {
        #     "motor.left.target": [int(l_speed)],
        #     "motor.right.target": [int(r_speed)],
        # }
        # aw(self.node.set_variables(v))
        self.dir = dir
        self.x = checkpoint[0]
        self.y = checkpoint[1]

    def get_motor_speeds(self):
        """
        Returns the current speeds of the left and right motors.
        
        Outputs:
        - left_speed: Speed of the left motor
        - right_speed: Speed of the right motor
        """
    # try:
        aw(self.node.wait_for_variables({"motor.left.speed", "motor.right.speed"}))
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
