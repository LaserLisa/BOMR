f'''
Extended_Kalman_Filter.py
'''
import math
import numpy as np


class Extended_Kalman_Filter():
    def __init__(self, pix2mm, robot_pose_px, time):
        '''
        Extended Kalman Filter Noise Covariance matrix.
        '''
        # Covariance for EKF simulation
        self.dt = 0    #[s]
        self.old_time = time 
        self.pix_to_mm = pix2mm #[mm/pxl]
        self.wheel_distance = 100
        pxl_var = 0.25
        self.Sigma = np.eye(5)  # Initialize covariance matrix
        self.Mu = [robot_pose_px[0][0], robot_pose_px[0][1], robot_pose_px[1], 0, 0]
        self.R = np.diag([pxl_var,  # variance of location on x-axis in pxl^2
                    pxl_var,   # variance of location on x-axis in pxl^2
                    0,         # variance of yaw angle          in rad^2
                    5.02,      # variance of velocity           in pxl^2/s^2
                    0])        # variance of angular velocity   in rad^2/s^2(yaw rate)

        self.Q = np.diag([0.04,    # variance of location on x-axis in pxl^2
                     0.04,    # variance of location on y-axis in pxl^2
                     0,       # variance of yaw angle          in rad^2
                     5.02,    # variance of velocity           in pxl^2/s^2
                     0])      # variance of angular velocity   in rad^2/s^2(yaw rate)


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
        Function that determines the current process noise covariance 
        Output: - Q : 5x5 covariance matrix based on estimated variance of different state parameters.
        '''
        Q = np.diag([0.04,   # variance of location on x-axis in pxl^2
                    0.04,   # variance of location on y-axis in pxl^2
                    0,       # variance of yaw angle          in rad^2
                    5.02,    # variance of velocity           in pxl^2/s^2
                    0     # variance of angular velocity   in rad^2/s^2(yaw rate)
                    ])
        return Q

    def U(self,speed_l,speed_r):
        '''
        Function that takes the raw speed values of each wheel motor and converts it into
        translation speed and rotation speed with a pixel scaling factor.

        Inputs: - speed_l: speed of left motor in mm/s
                - speed_r: speed of right motor in mm/s
                - pix_to_mm : in pxl/mm

        Output: - u  : The current motor control values.
                       1x2 vector that holds current inputed speed and angular velocity.
        '''
        v = (1/self.pix_to_mm) * (speed_r + speed_l)/2
        theta_dot = (speed_r - speed_l)/(self.wheel_distance + 11) 
        u = np.array([v, theta_dot]).T
        return u

    def measure_state(self,x):
        '''
        Returns the observed system state based on prior state estimates
        and noise characteristics.

        Inputs:
        previous_state: A 1x5 vector representing the prior estimates of 
                        position (x, y), orientation (theta), velocity, 
                        and angular velocity.

        Outputs:
        A 1x5 vector representing the observed state values derived 
        from measurements, incorporating noise.
        '''
        C = np.eye(5)
        y = np.dot(C,x) + np.diag(self.R)
        return y

    def system_state(self, robot_pose_px):
        '''
        Computes the current measurement state vector based on the robot's observed pose.

        Inputs:
        robot_pose_px: A tuple containing the robot's position (x, y) and orientation (angle) 
                       as observed from external measurements.

        Outputs:
        y_current: A 1x5 vector representing the current measured state, which includes:
                   position (x, y), orientation (angle), velocity, and angular velocity.
    
        '''
        if (self.dt == 0):
            self.dt = 1
        
        y=np.zeros(5).T
        y[0] = robot_pose_px[0][0]
        y[1] = robot_pose_px[0][1]
        y[2] = robot_pose_px[1]
        y[3] = np.sqrt((y[0]-self.Mu[0])**2+(y[1]-self.Mu[1])**2)/self.dt
        y[4] = (y[2]-self.Mu[2])/self.dt
        y_current = self.measure_state(y)
        return y_current

    def Get_Mu_pred(self, u, Q, delta_t, Mu):
        '''
        Predicts the next system state based on the previous state and control input.

        Args:
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

        Returns:
        Mu_pred: A 1x5 vector representing the predicted system state, which includes:
                 predicted position, orientation, velocity, and angular velocity.
        '''
        
        theta = Mu[2]
        A = np.eye(5)*np.array([1,1,1,0,0])
        B = np.array([[delta_t*math.cos(theta),0],
                     [-delta_t*math.sin(theta),0],
                     [0,delta_t],
                     [1,0],
                     [0,1]])
        Mu_pred = np.dot(A,Mu) + np.dot(B,u) + np.diag(Q)
        return Mu_pred

    def jacobian_G(self,theta, v):
        '''
        Function that returns the Jacobian of the thymio state function applied to the previous system state estimations.

        Inputs: - theta : the current estimated Thymio angle in rad
                - v  : The current motor control speed in pxl/s

        Output: - G : 5x5 Jacobian matrix of thymio state function applied to input values
        '''
        v = 1/(self.pix_to_mm) * v
        G = np.array([[1,0,-self.dt*v*math.sin(theta),self.dt*math.cos(theta),0],
                     [0,1, self.dt*v*math.cos(theta),self.dt*math.sin(theta),0],
                     [0,0,1,0,self.dt],
                     [0,0,0,1,0],
                     [0,0,0,0,1]])
        return G


    def extended_kalman(self, u, y):
        '''
        Function that takes the previous mean and covariance estimations and Applies an extended Kalman Filter.
        It incorporates the camera mesurements and motor controls to find the current Kalman gain.
        Kalman gain is then used to correct system state estimations.

        Inputs: - Mu    : The previous mesurement time's mean system state values.
                          1x5 vector that holds the current axis coordinates, angle,
                          velocity and angular velocity estimations.
                - Sigma : The previous measurement time's Covariance matrix.
                          5x5 matrix and holds each the covariance values of the previous estimations.
                - u     : The current motor control values.
                          1x2 vector that holds current inputed speed and angular velocity.
                - y     : The current system camera measurements.
                          1x5 vector that holds measured axis coordinates, angle based on camera measurements,
                          as well as the velocity and angular velocity based on previous velocity estimations.

        Outputs: - Mu_est    : The current estimated mean system state values, after Kalman filtering.
                               1x5 vector that holds the same data types as the input Mu.
                 - Sigma_est : The current estimated System Covariance matrix.
                               5x5 matrix that holds each covariance value of current state estimations.
        '''
        #Predict values
        Q = self.process_cov()
        Mu_pred = self.Get_Mu_pred(u, Q, self.dt, self.Mu)
        G = self.jacobian_G(self.Mu[2], u[0])
        Sigma_pred = np.dot(G,np.dot(self.Sigma,G.T)) + Q
        
        # Check if the camera measurement is valid (i.e., no NaN values)
        if np.isnan(y).any():
            #print("Camera measurement unavailable, skipping update step.")
            # Skip the measurement update step and only return the prediction
            Mu_est = Mu_pred  # No update to the state from the camera
            Sigma_est = Sigma_pred  # No update to the covariance
        else: 
            #Calculate Kalman Gain
            H = np.eye(5)  #technically jacobian of h(x), the camera measurements function
            S = np.dot(np.dot(H,Sigma_pred),H.T) + self.R
            K = np.dot(Sigma_pred,np.dot(H.T,np.linalg.inv(S)))
        
            #Update Estimated values
            Mu_est = Mu_pred + np.dot(K,(y - self.measure_state(Mu_pred)))
            Sigma_est = np.dot((np.eye(5)-np.dot(K,H)),Sigma_pred)+np.eye(5)
        # Sigma_est[Sigma_est < 1e-5] = 0
        self.Mu, self.Sigma = Mu_est, Sigma_est
        self.Mu[3], self.Mu[4] = 0,0 

    def Kalman_main(self, l_speed, r_speed, time, robot_pose_px):
        self.update_time (time)
        self.extended_kalman(self.U(l_speed, r_speed),self.system_state(robot_pose_px))
        x, y, theta = self.Mu[0], self.Mu[1], self.Mu[2]
        robot_pose = (np.array([x, y]), theta)
        return robot_pose
         
    def get_robot_pose(self)-> tuple:
        x, y, theta = self.Mu[0], self.Mu[1], self.Mu[2]
        return (np.array([x, y]), theta)
