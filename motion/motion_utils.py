import numpy as np
import math

# Mode rotation rapide : Kp_angle = 4, Kp_dist = 0.5


class motors_regulator:
    Kp_angle = 1
    Kp_dist = 1

class robot_position:
    actual_pos = [0, 0]
    alpha = 0.0
    x = 0.0
    y = 0.0
    area_radius = 10

def motors(left, right):
    return {
        "motor.left.target": [int(left)],
        "motor.right.target": [int(right)],
    }

def compute_distance(actual_point, point_to_go):
  distance = math.sqrt(pow(actual_point[0] - point_to_go[0], 2) + pow(actual_point[1] - point_to_go[1], 2))
  return distance
  
def compute_angle(actual_point, point_to_go):
  angle = -180*math.atan2(point_to_go[1] - actual_point[1], point_to_go[0] - actual_point[0])/math.pi
  return angle

def compute_regulator_gain(distance, distance_tot):
  if distance < 40 or (distance_tot - distance) < 40:
      Kp_angle = 4
      Kp_dist = 0.5

  elif distance > 40 and (distance_tot - distance) > 40:
      Kp_angle = 1
      Kp_dist = 1

  return Kp_angle, Kp_dist

def compute_motor_speed(angle_error, regulator, is_finished):
# distance_tot is the total distance between the two actual points 

  if is_finished:
    motor_L = 0
    motor_R = 0
    return motor_L, motor_R

  elif angle_error < -180 :
    angle_error = 360 + angle_error

  elif angle_error > 180 :
    angle_error = -360 + angle_error
      
  #motor_L = (regulator.Kp_dist * (180-abs(angle_error)))**2 + regulator.Kp_angle * angle_error
  #motor_R = (regulator.Kp_dist * (180-abs(angle_error)))**2 - regulator.Kp_angle * angle_error

  motor_L = (regulator.Kp_dist * (180-abs(angle_error))) + regulator.Kp_angle * angle_error
  motor_R = (regulator.Kp_dist * (180-abs(angle_error))) - regulator.Kp_angle * angle_error


  return motor_L, motor_R

def set_point_to_go(center, actual_point, prev_point_to_go, point_to_go, global_trajectory, distance, is_finished):
  if actual_point == 0:
    point_to_go = list(global_trajectory[0])
    actual_point = 1
    prev_point_to_go = center


  # Point to go = Position of the robot if we reach the end of the list
  elif actual_point >= len(global_trajectory) and distance < 10:
    is_finished = True

  # If the robot is close to the point to go -> Next point
  elif distance < 10:
    prev_point_to_go = point_to_go
    point_to_go = list(global_trajectory[actual_point])
    actual_point += 1

  return actual_point, point_to_go, prev_point_to_go, is_finished


