def kalmanfilter(x_prev, P_prev, x_cam, y_cam, theta_cam, r_speed, l_speed, cam_up, prev_time):
    """
    Calculate new position and orientation  
    :param x_prev: The last state vector computed from the filter
    :param P_prev: Covariance matrix of predicted state
    :param x_cam, y_cam, theta_cam: Position in x,y,theta measured by camera
    :param r_speed, l_speed: current speed of right and left motors 
    :param prev_time: time where the last execution ended
    
    :return x_est: a posteriori estimated state
    :return P_est: Covariance of matrix of estimated state
    """
        
    #Constant initialization
        
    lth=95 # Length between two wheels of the thymio [mm]
    speed_conv = 0.332 # Conversion factor from motor speed to mm/s (thymio 635)
    pixel_conv =0.9 # Conversion factor from mm to pixels (1.3 DLL table mi haute)     
    
    #Initialization of covariance matrix R
    r_px=0.05
    r_py=0.05
    r_theta=0.01

    #Iniialization of covariance matrix Q
    q_px=0.25
    q_py=0.25
    q_theta=0.1
    Q = np.array([[q_px, 0, 0],
                  [0, q_py, 0],
                  [0, 0, q_theta]])
    
    #Matrix A
    A=np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
    
    #Make sure we're not missinformed by camera (occurs once when hidding it)
    if x_cam - x_prev[0, 0] > 50 or y_cam - x_prev[1,0] > 50 or x_cam - x_prev[0, 0] < -50 or y_cam - x_prev[1,0] < -50:
        x_cam = x_prev[0, 0]
        y_cam = x_prev[1, 0]
        theta_cam = x_prev[2, 0]
    
    Ts=time.time()-prev_time #Get new sampling time 
    #Convert from degrees to radians 
    theta_cam=math.radians(theta_cam)
    x_prev[2,0] = math.radians(x_prev[2,0])
    theta = x_prev[2,0] 

    # Matrix B
    B=np.array([[pixel_conv*math.cos(theta)*Ts/2, pixel_conv*math.cos(theta)*Ts/2],
                [-pixel_conv*math.sin(theta)*Ts/2, -pixel_conv*math.sin(theta)*Ts/2],
                [Ts/lth, -Ts/lth]])
    
    # Input Matrix
    u=np.array([[r_speed*speed_conv],
                [l_speed*speed_conv]])
    
    Ts=time.time()-prev_time # Get new sampling time 
    
    #Prediciton of the next state
    x_est_a_priori = np.dot(A, x_prev)+np.dot(B, u)

    #Make sure the new angle is between -pi and pi
    if(x_est_a_priori[2,0]>math.pi):
        x_est_a_priori[2,0]=x_est_a_priori[2,0]-2*math.pi
    elif(x_est_a_priori[2,0]>math.pi):
        x_est_a_priori[2,0]=x_est_a_priori[2,0]+2*math.pi 
    
    P_est_a_priori = np.dot(A, np.dot(P_prev, A.T))
    P_est_a_priori = P_est_a_priori + Q

    #Update         
    if(cam_up==True):
        #Camera is measuring position and orientation
        y=np.array([[x_cam],
                    [y_cam],
                    [theta_cam]])
        H=np.array([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
        R=np.array([[r_px, 0, 0],
                    [0, r_py, 0],
                    [0, 0, r_theta]])
        
        # innovation calculation
        i = y - np.dot(H, x_est_a_priori)

        # measurement prediction covariance
        S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R

        # Kalman gain 
        K = np.dot(P_est_a_priori, np.dot(H.T, np.linalg.inv(S)))

        # a posteriori estimate
        x_est = x_est_a_priori + np.dot(K,i)
        P_est = P_est_a_priori - np.dot(K,np.dot(H, P_est_a_priori))
        
    else:
        #No camera information, No measurement available 
        x_est = x_est_a_priori
        P_est = P_est_a_priori
    
    x_est[2, 0] = math.degrees(x_est[2, 0]) # Return degrees to the main code
    new_time=time.time() # Save time for next execution
    
    return x_est, P_est, new_time