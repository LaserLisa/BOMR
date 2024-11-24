print(">> Initializing filter") 
EKF = Kalman_Filter.Extended_Kalman_Filter()
EKF.Sigma = np.eye(5)
EKF.Mu = [x,y,angle,0,0] #initial values for position and orientation
EKF.input_speed=CALIBRATE_FACTOR*50

#when calling it 
EKF.Mu = [x,y,angle,0,0]
# finding the control input for motor speed 
motor_values =EKF.u_input(-direction*TURNING_SPEED*CALIBRATE_FACTOR,-direction*TURNING_SPEED*CALIBRATE_FACTOR, DISTANCE_WHEELS,ur.scaling_factor)


print(">>> Filtering")
#if robot is turning to it's new goal 
  EKF.dt = time.time()-time_stamp
  EKF.extended_kalman(motor_values,EKF.capture_measurements(ur))
  new_x, new_y, new_angle = (EKF.Mu[0],EKF.Mu[1]), EKF.Mu[2]

#if robot is moving to it's new goal 
  EKF.dt = time.time()-time_stamp
  EKF.extended_kalman(motor_values,EKF.capture_measurements(ur))
  new_x, new_y, new_angle = (EKF.Mu[0],EKF.Mu[1]), EKF.Mu[2]

#if robot is detecting an obstable  
  EKF.dt = time.time()-time_stamp
  EKF.extended_kalman(motor_values,EKF.capture_measurements(ur))
  new_x, new_y, new_angle = (EKF.Mu[0],EKF.Mu[1]), EKF.Mu[2]


