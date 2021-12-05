def obstacle_avoidance_speed(prox):
  obstThr = 1000
  obstacleGain = 0.05
  speed0 = 400
  obstacle_detected = False
  
  for i in range(1, 4, 1):
    if(prox[i] > obstThr):
      obstacle_detected = True

  if not(obstacle_detected) :
    return 0, 0

  elif obstacle_detected :
    diffDelta = 0.5*prox[0] + 0.75*prox[1] - 1*prox[2] -0.75*prox[3] - 0.5*prox[4]
    motor_L = obstacleGain * diffDelta 
    motor_R = - obstacleGain * diffDelta
    return motor_L, motor_R
