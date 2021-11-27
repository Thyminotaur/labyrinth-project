# ----------------------------------------
# Obstacle avoidance main python script
# ----------------------------------------
from tdmclient import ClientAsync

# ---------------------------------------
# Motor definition for this code
# ---------------------------------------
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

# ---------------------------------------
# def obstacle_avoidance(node, variables, speedLeft, speedRight):
# ---------------------------------------
def obstacle_avoidance_full(node, variables):
    # global motor_left, motor_right
    try:   
        speed0 = 100       		# nominal speed
        speedGain = 5      		# gain used with ground gradient
        obstacleGain = 2        # gain used to avoid obstacle
        obstThr = 20        	# obstacle threshold to switch state 1->0
        state = 0          		# 0=gradient, 1=obstacle avoidance
        prox = variables["prox.horizontal"]

        # difference between sensors
        diffDelta = prox[0]+0.5*prox[1]-0.25*prox[2]-0.5*prox[3]-prox[4]

        if state == 0 and (prox[0] > obstThr or prox[1] > obstThr or prox[2] > obstThr or prox[3] > obstThr or prox[4] > obstThr):
        # switch from goal tracking to obst avoidance if obstacle detected
           state = 1

        elif state == 1 and (prox[0] < obstThr and prox[1] < obstThr and prox[2] < obstThr and prox[3] < obstThr and prox[4] < obstThr):
        # switch from obst avoidance to goal tracking if obstacle got unseen
             state = 0

             if state == 0 :
             # goal tracking: turn toward the goal
                motor_left = speed0 - speedGain * diffDelta
                motor_right = speed0 + speedGain * diffDelta           
                
             else :
             # obstacle avoidance: accelerate wheel near obstacle
                motor_left = speed0 + obstacleGain* diffDelta #(obst[0] / 100)
                motor_right = speed0 + obstacleGain* (-diffDelta) # (obst[4] / 100)

        node.send_set_variables(motors(motor_left, motor_right))
    except KeyError:
        pass  # prox.horizontal not found

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.watch(variables=True)
            node.add_variables_changed_listener(obstacle_avoidance_full)
            await client.sleep()
    client.run_async_program(prog)
