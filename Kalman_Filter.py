'''
Extended_Kalman_Filter.py
'''
import math
import numpy as np

class Extended_Kalman_Filter():
    def __init__(self):
        '''
        Extended Kalman Filter Noise Covariance matrix.
        R is the measurement noise covariant matrix. It injects the standard deviation of correct thymio measurement localization
        by taking into account pixel resolution, image scale and computational errors from our find_robot function.
        '''
        # Covariance for EKF simulation
        self.dt = 1     #[s]
        self.input_speed = 14 #[mm/s]
        self.scaling_factor = 3 #[mm/pxl]
        pxl_var = 0.25
        self.R = np.diag([pxl_var,  # variance of location on x-axis in pxl^2
                    pxl_var,   # variance of location on x-axis in pxl^2
                    0,         # variance of yaw angle          in rad^2
                    6.15,      # variance of velocity           in pxl^2/s^2
                    0])        # variance of angular velocity   in rad^2/s^2(yaw rate)

        self.Q = np.diag([0.04,    # variance of location on x-axis in pxl^2
                     0.04,    # variance of location on y-axis in pxl^2
                     0,       # variance of yaw angle          in rad^2
                     6.15,    # variance of velocity           in pxl^2/s^2
                     0])      # variance of angular velocity   in rad^2/s^2(yaw rate)

    def process_cov(self):
        '''
        Function that determines the current process noise covariance depending on scaling factor.
        It uses the estimated standard diviation of the motor speeds to determine the process noise expected
        on thymio predicted system states.

        Input: - scaling_factor : scalar parameter determined by measured pixel distance between identifier dots in pxl
                                  on the Thymio and the actual real distance in mm.
        Output: - Q : 5x5 covariance matrix based on estimated variance of different state parameters.
        '''
        Q = np.diag([0.04,   # variance of location on x-axis in pxl^2
                    0.04,   # variance of location on y-axis in pxl^2
                    0,       # variance of yaw angle          in rad^2
                    6.15,    # variance of velocity           in pxl^2/s^2
                    0     # variance of angular velocity   in rad^2/s^2(yaw rate)
                    ])
        return Q

    def u_input(self,speed_l,speed_r, wheel_distance, scaling_factor):
        '''
        Function that takes the raw speed values of each wheel motor and converts it into
        translation speed and rotation speed with a pixel scaling factor.

        Inputs: - speed_l: speed of left motor in mm/s
                - speed_r: speed of right motor in mm/s
                - scaling_factor : in pxl/mm

        Output: - u  : The current motor control values.
                       1x2 vector that holds current inputed speed and angular velocity.
        '''
        v = self.scaling_factor * (speed_r + speed_l)/2
        theta_dot = (speed_r - speed_l)/wheel_distance
        u = np.array([v, theta_dot]).T
        return u

    def measure_state(self,x):
        '''
        Function that returns the current measured state values depending on previous system state values
        and current input motor control vector.

        Inputs: - x : The previous mesurement time's mean system state values.
                      1x5 vector that holds the current axis coordinates, angle,
                      velocity and angular velocity estimations/observations.

        Outputs: - y : The current measured system state values based on observation values
                       1x5 vector that holds the current axis coordinates, angle,
                       velocity and angular velocity mesurements.
        '''
        C = np.eye(5)
        y = np.dot(C,x) + np.diag(self.R)
        return y

    def capture_measurements(self,ur):
        '''
        Function that takes a picture and analyses the data to return the current measurement state vector.

        Inputs: - Mu : The previous mesurement time's mean system state values.
                   1x5 vector that holds the current axis coordinates, angle,
                   velocity and angular velocity estimations.

        Output: - y_current : The current measured system state values based on observation values
                              1x5 vector that holds the current axis coordinates, angle,
                              velocity and angular velocity mesurements.
        '''
        y=np.zeros(5).T
        y[0] = ur.coords[0]
        y[1] = ur.coords[1]
        y[2] = ur.angle
        y[3] = math.sqrt((y[0]-self.Mu[0])**2+(y[1]-self.Mu[1])**2)/self.dt
        y[4] = (y[2]-self.Mu[2])/self.dt
        y_current = self.measure_state(y)
        return y_current

    def thymio_state(self, u, Q, delta_t, Mu):
        '''
        Function that returns the current system state values depending on previous system state values
        and current input motor control vector.

        Inputs: - Mu : The previous mesurement time's mean system state values.
                       1x5 vector that holds the current axis coordinates, angle,
                       velocity and angular velocity estimations.
                - u  : The current motor control values.
                       1x2 vector that holds current inputed speed and angular velocity.

        Outputs: -Mu_pred : The current predicted system state values based on previous values
                            and current input motor control data.
        '''
        theta = Mu[2]
        A = np.eye(5)*np.array([1,1,1,0,0])
        B = np.array([[delta_t*math.cos(theta),0],
                     [delta_t*math.sin(theta),0],
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
        G = np.array([[1,0,-self.dt*v*math.sin(theta),self.dt*math.cos(theta),0],
                     [0,1, self.dt*v*math.cos(theta),self.dt*math.sin(theta),0],
                     [0,0,1,0,self.dt],
                     [0,0,0,1,0],
                     [0,0,0,0,1]])
        return G

    def get_Mu_pred(self, u, delta_t, ur):
        '''
        Function that returns the actual value of 'Mu_pred'.

        Inputs: - u       : The current motor control values.
                            1x2 vector that holds current inputed speed and angular velocity.
                - delta_t : Time interval.
                - ur      : utilsRobot instance.

        Output: - Mu_pred : Predicted value of 'Mu_pred'.
        '''
        Q = self.process_cov()
        Mu_pred = self.thymio_state(u, Q, delta_t, np.array([ur.coords[0], ur.coords[1], ur.angle, 0, 0]).T)
        return Mu_pred

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
        Mu_pred = self.thymio_state(u, Q, self.dt, self.Mu)
        G = self.jacobian_G(self.Mu[2], u[0])
        Sigma_pred = np.dot(G,np.dot(self.Sigma,G.T)) + Q

        #Calculate Kalman Gain
        H = np.eye(5)  #technically jacobian of h(x), the camera measurements function
        S = np.dot(np.dot(H,Sigma_pred),H.T) + self.R
        K = np.dot(Sigma_pred,np.dot(H.T,np.linalg.inv(S)))

        #Update Estimated values
        Mu_est = Mu_pred + np.dot(K,(y - self.measure_state(Mu_pred)))
        Sigma_est = np.dot((np.eye(5)-np.dot(K,H)),Sigma_pred)+np.eye(5)*1.00001
        # Sigma_est[Sigma_est < 1e-5] = 0
        self.Mu, self.Sigma = Mu_est, Sigma_est
