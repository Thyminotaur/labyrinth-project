from tdmclient import ClientAsync, aw
import time

def motors(left, right):
    return {
        "motor.left.target": [int(left)],
        "motor.right.target": [int(right)],
    }

client = ClientAsync()

print("\nWaiting for node ..")
node = aw(client.wait_for_node())
print("\nNode : " + str(node))

i = 0
max_count = 200
prox_two = 0
diffDelta = 0
state = 0
obstThr = 3000
obstacleGain = 0.05
speed0 = 400 

aw(node.lock())

while i<max_count:
  #aw(node.lock())
  aw(node.wait_for_variables({"prox.horizontal"}))
  #time.sleep(0.01)
    
  #prox = node.v.prox.horizontal
  if(i > 100):
      speed_motor_r = node.v.motor.right.speed
      speed_motor_l = node.v.motor.left.speed
      print(str(speed_motor_r) + "\t" + str(speed_motor_l))

##  diffDelta = 0.5 * prox[0]+0.75*prox[1]-prox[2]-0.75*prox[3]- 0.5*prox[4]
##    
##  if state == 0 and (prox[0] > obstThr or prox[1] > obstThr or prox[2] > obstThr or prox[3] > obstThr or prox[4] > obstThr):
##    # switch from goal tracking to obst avoidance if obstacle detected
##    state = 1
##
##  elif state == 1 and (prox[0] < obstThr and prox[1] < obstThr and prox[2] < obstThr and prox[3] < obstThr and prox[4] < obstThr):
##    # switch from obst avoidance to goal tracking if obstacle got unseen
##    state = 0
##
##  if state == 0 :
##    # goal tracking:
##    motor_left = speed0 + 0.1*(prox[5] + prox[6])
##    motor_right = speed0 + 0.1*(prox[5] + prox[6])       
##                
##  else :
##    # obstacle avoidance: accelerate wheel near obstacle
##    motor_left = speed0 + obstacleGain* diffDelta 
##    motor_right = speed0 + obstacleGain* (-diffDelta)

  aw(node.set_variables(motors(100, 100)))

  #print("\n" + str(prox_two))
  i = i+1

print("\nRun finished")
aw(node.set_variables(motors(0, 0)))
aw(node.unlock())



