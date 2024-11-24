print(">> Initializing filter") 
EKF = Kalman_Filter.Extended_Kalman_Filter()
EKF.Sigma = np.eye(5)
EKF.Mu = [x,y,angle,0,0] #initial values for position and orientation
EKF.input_speed=CALIBRATE_FACTOR*50

#when calling it 
EKF.Mu = [x,y,angle,0,0]
u=EKF.u_input(-direction*TURNING_SPEED*CALIBRATE_FACTOR,-direction*TURNING_SPEED*CALIBRATE_FACTOR, DISTANCE_WHEELS,ur.scaling_factor)
