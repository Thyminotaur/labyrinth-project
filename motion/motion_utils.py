import numpy as np
import math

class motors_regulator:
    Kp_angle = 4
    Kp_dist = 0.5

class robot_position:
    actual_pos = [0, 0]
    alpha = 0.0

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def compute_distance(actual_point, point_to_go):
  distance = math.sqrt(pow(actual_point[0] - point_to_go[0], 2) + pow(actual_point[1] - point_to_go[1], 2))
  return distance
  
def compute_angle(actual_point, point_to_go):
  angle = -180*math.atan2(point_to_go[1] - actual_point[1], point_to_go[0] - actual_point[0])/math.pi
  return angle

def compute_motor_speed(angle_error, regulator, is_finished):
  if is_finished:
    motor_L = 0
    motor_R = 0

  elif alpha_e < -180 :
    alpha_e = 360 + alpha_e

  elif alpha_e > 180 :
    alpha_e = -360 + alpha_e

  motor_L = regulator.Kp_dist * (180-abs(alpha_e)) + regulator.Kp_angle * alpha_e
  motor_R = regulator.Kp_dist * (180-abs(alpha_e)) - regulator.Kp_angle * alpha_e

  return motor_L, motor_R

def set_point_to_go(actual_point, global_trajectory, distance, is_finished):
  if actual_point == 0:
    point_to_go = list(global_trajectory[0])
    actual_point = 1

  # Point to go = Position of the robot if we reach the end of the list
  elif actual_point >= len(global_trajectory) and distance < 5:
    is_finished = True

  # If the robot is close to the point to go -> Next point
  elif distance < 5:
    point_to_go = list(global_trajectory[actual_point])
    actual_point += 1

  return actual_point


