f'''
Extended_Kalman_Filter.py
'''
import math
import numpy as np

class Extended_Kalman_Filter():
    def __init__(self, pix2mm, robot_pose_px, time):

        self.dt = 0    #[s]
        self.old_time = time 
        self.Pix_to_mm = pix2mm #[mm/pxl]
        self.wheel_distance = 100
        self.Sigma = np.eye(5)  
        self.Mu = [robot_pose_px[0][0], robot_pose_px[0][1], robot_pose_px[1], 0, 0]
        self.R = np.diag([0.25,  # var of x in pxl^2
                    0.25,   # var of y in pxl^2
                    0,        
                    5.02,      # var of v in pxl^2/s^2
                    0])        

        self.Q = np.diag([0.04,    # var of x in pxl^2
                     0.04,    # var of y in pxl^2
                     0,       
                     5.02,    # var of v in pxl^2/s^2
                     0])      


    def update_time(self, time):
        '''
        Function that determines the amount of time in seconds since the last call of the kalman filter 
        Input: - time : time of execution of the EKF
        Output: - dt : change in time since last iteration 
                - olt_time : save the value of the timer for next iteration 
        '''
        self.dt = time - self.old_time
        self.old_time = time 
        
        
    def process_cov(self):
        '''
        Function that outputs the process noise covariance
        Output: - Q : 5x5 covariance matrix based on estimated variance of different state parameters.
        '''
     
        return self.Q

    def U(self,speed_l,speed_r):
        '''
        Function that convert motor values to speed and angular velocity

        Inputs: - speed_l: speed of left motor in mm/s
                - speed_r: speed of right motor in mm/s
                - Pix_to_mm : in pxl/mm

        Output: - u  : The motor control values 
        '''
        v = (1/self.Pix_to_mm) * (speed_r + speed_l)/2
        theta_dot = (speed_r - speed_l)/(self.wheel_distance + 11) 
        u = np.array([v, theta_dot]).T
        return u

    def observe_current_state(self, previous_state):
        """
        Returns the observed system state based on prior state estimates
        and noise characteristics.

        input:
            previous_state: A 1x5 vector representing the prior estimates of 
                            position (x, y), orientation (theta), velocity, 
                            and angular velocity.

        output:
            A 1x5 vector representing the observed state values derived 
            from measurements, incorporating noise.
        """
        observation_matrix = np.eye(5)
        observed_state = np.dot(observation_matrix, previous_state) + np.diag(self.R)
        return observed_state

    def system_state(self, robot_pose_px):
        """
        Computes the current measurement state vector based on the robot's observed pose.

        input:
            robot_pose_px: A tuple containing the robot's position (x, y) and orientation (angle) 
                          as observed from external measurements.

        output:
             y_current: A 1x5 vector representing the current measured state, which includes:
                      position (x, y), orientation (angle), velocity, and angular velocity.
        """
        # Ensure a non-zero time step to prevent division errors
        if self.dt == 0:
            self.dt = 1

        # Initialize the state vector
        y = np.zeros(5).T

        # Update the position (x, y) and orientation (theta) from the observed pose
        y[0] = robot_pose_px[0][0]  # x-coordinate
        y[1] = robot_pose_px[0][1]  # y-coordinate
        y[2] = robot_pose_px[1]     # orientation (theta)

        # Compute velocity as the distance traveled divided by the time step
        y[3] = np.sqrt((y[0] - self.Mu[0]) ** 2 + (y[1] - self.Mu[1]) ** 2) / self.dt

        # Compute angular velocity as the change in orientation divided by the time step
        y[4] = (y[2] - self.Mu[2]) / self.dt

        # Adjust the state vector based on observed measurements and noise characteristics
        y = self.observe_current_state(y)

        return y


    def Get_Mu_pred(self, u, Q, delta_t, Mu):
        """
        Predicts the next system state based on the previous state and control input.

        input:
            Mu: A 1x5 vector representing the previous system state, which includes:
                - x, y: Current position
                - theta: Orientation angle
                - velocity (v): Linear velocity
                - angular velocity (omega): Angular velocity
            u: A 1x2 vector containing the current motor control inputs:
                - Linear speed
                - Angular velocity
            Q: A diagonal matrix representing the process noise covariance.
            delta_t: The time step between the previous and current state.

        output:
            Mu_pred: A 1x5 vector representing the predicted system state, which includes:
                        predicted position, orientation, velocity, and angular velocity.
        """
        # Extract the current orientation angle
        theta = Mu[2]

        # Define the state transition matrix A
        A = np.eye(5) * np.array([1, 1, 1, 0, 0])

        # Define the control input matrix B
        B = np.array([
            [delta_t * math.cos(theta), 0],  # Effect on x
            [-delta_t * math.sin(theta), 0],  # Effect on y
            [0, delta_t],  # Effect on theta
            [1, 0],  # Effect on velocity
            [0, 1]   # Effect on angular velocity
        ])

        # Compute the predicted state
        Mu_pred = np.dot(A, Mu) + np.dot(B, u) + np.diag(Q)

        return Mu_pred

    def get_G(self, theta, v):
        """
        Computes the Jacobian matrix for the system's state transition function, 
        applied to the given state parameters.

        Inputs:
            theta: The current estimated angle of the robot (in radians).
            v: The current speed of the robot's motor control (in pixels/second).

        Outputs:
            G: A 5x5 Jacobian matrix representing the linearized state transition 
                function for the given inputs.
        """
        # Convert speed from pixels/second to millimeters/second
        v_mm = v / self.Pix_to_mm

        # Define the Jacobian matrix G
        G = np.array([
            [1, 0, -self.dt * v_mm * math.sin(theta), self.dt * math.cos(theta), 0],
            [0, 1,  self.dt * v_mm * math.cos(theta), self.dt * math.sin(theta), 0],
            [0, 0,  1,                                 0,                        self.dt],
            [0, 0,  0,                                 1,                        0],
            [0, 0,  0,                                 0,                        1]
        ])

        return G



    def Kalman_filter(self, u, y):
        '''
        This function applies an Extended Kalman Filter (EKF) to refine the system's state estimations using the 
        previous mean and covariance estimates, current motor control inputs, and camera measurements.

        The process involves calculating the Kalman gain based on the given inputs and using it to correct 
        the system's state estimates.

        Inputs:
        - Mu: The mean system state vector from the previous timestep.
                A 1x5 vector containing values for the axis coordinates, angle, velocity, 
                and angular velocity.
        - Sigma: The covariance matrix from the previous timestep.
                    A 5x5 matrix representing the covariance of the state estimates.
        - u: The current motor control input.
                A 1x2 vector that includes the input speed and angular velocity.
        - y: The current system measurements from the camera.
                A 1x5 vector containing measured values for axis coordinates, angle, velocity, 
                and angular velocity based on observations and prior velocity estimates.

        Outputs:
        - Mu_est: The updated mean system state vector after applying the Kalman filter.
                    A 1x5 vector with the same data structure as the input Mu.
        '''

        #Predict values
        Mu_pred = self.Get_Mu_pred(u, self.Q, self.dt, self.Mu)
        G = self.get_G(self.Mu[2], u[0])
        Sigma_pred = np.dot(G,np.dot(self.Sigma,G.T)) + self.Q
        
        # Check if the camera measurement is valid (i.e., no NaN values)
        if np.isnan(y).any():
            #print("Camera measurement unavailable, skipping update step.")
            # Skip the measurement update step and only return the prediction
            #Mu[2] = self.orientation_tracker.value
            Mu_est = Mu_pred  # No update to the state from the camera
            Sigma_est = Sigma_pred  # No update to the covariance
        else: 
            #Calculate Kalman Gain, based on Serie 8 
            H = np.eye(5)  
            S = np.dot(np.dot(H,Sigma_pred),H.T) + self.R
            K = np.dot(Sigma_pred,np.dot(H.T,np.linalg.inv(S)))
        
            #Update Estimated values
            Mu_est = Mu_pred + np.dot(K,(y - self.observe_current_state(Mu_pred)))
            Sigma_est = np.dot((np.eye(5)-np.dot(K,H)),Sigma_pred)
        self.Mu, self.Sigma = Mu_est, Sigma_est
        self.Mu[3], self.Mu[4] = 0,0 

    def Kalman_main(self, l_speed, r_speed, time, robot_pose_px):
        self.update_time (time)
        self.Kalman_filter(self.U(l_speed, r_speed),self.system_state(robot_pose_px))
        x, y, theta = self.Mu[0], self.Mu[1], self.Mu[2]
        robot_pose = (np.array([x, y]), theta)
        return robot_pose
         
    def get_robot_pose(self)-> tuple:
        x, y, theta = self.Mu[0], self.Mu[1], self.Mu[2]
        return (np.array([x, y]), theta)
