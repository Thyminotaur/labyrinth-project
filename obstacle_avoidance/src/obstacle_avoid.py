# Obstacle avoidance main python script
from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }


client = ClientAsync()
#def obstacle_avoidance(node, variables, speedLeft, speedRight):
def obstacle_avoidance(node, variables):
    
    # initialisation variable
    try:

        speed0 = 100       		# nominal speed
        #nospeed = 0         	# no speed
        speedGain = 5      		# gain used with ground gradient
        obstThr = 10        	# obstacle threshold to switch state 1->0
        obstSpeedGain = 2  	    # /100 (actual gain: 5/100=0.05)
        state = 0          		# 0=gradient, 1=obstacle avoidance
        prox = variables["prox.horizontal"]
        #prox_far_left = prox[0]
        #prox_left = prox[1]
        #prox_front = prox[2]
        #prox_right = prox[3]
        #prox_far_right = prox[4]

        # acquisition from the proximity sensors to detect obstacles
        obst = [prox.horizontal[0], prox.horizontal[1], prox.horizontal[2], prox.horizontal[3],prox.horizontal[4]]

        # difference between sensors
        diffDelta = prox.horizontal[0]+0.5*prox.horizontal[1]-0.25*prox.horizontal[2]-0.5*prox.horizontal[3]-prox.horizontal[4]

        if state == 0 and (obst[0] > 2*obstThr or obst[1] > 2*obstThr or obst[2] > 2*obstThr or obst[3] > 2*obstThr or obst[4] > 2*obstThr):
        # switch from goal tracking to obst avoidance if obstacle detected
            state = 1
        elif state == 1 and (obst[0] < obstThr and obst[1] < obstThr and obst[2] < obstThr and obst[3] < obstThr and obst[4] < obstThr):
        # switch from obst avoidance to goal tracking if obstacle got unseen
            state = 0
        
            if  state == 0 :
            # goal tracking: turn toward the goal
                #motor.left.target = speed0 - speedGain * diffDelta
                #motor.right.target = speed0 + speedGain * diffDelta
                motors.left = speed0 - speedGain * diffDelta
                motors.right = speed0 + speedGain * diffDelta
            else : # if state == 1, so there is an obstacle
            # obstacle avoidance: accelerate wheel near obstacle
                #motor.left.target = speed0 + obstSpeedGain * diffDelta #(obst[0] / 100)
                #motor.right.target = speed0 + obstSpeedGain *(-diffDelta) # (obst[4] / 100)
                motors.left = speed0 + obstSpeedGain * diffDelta #(obst[0] / 100)
                motors.right = speed0 + obstSpeedGain *(-diffDelta) # (obst[4] / 100)
       # speedLeft = motors.left
        #speedRight = motors.right 
        #speed_motor = (speedLeft, speedRight)
        #return speed_motor
        # node.send_set_variables(motors.left, motors.right) #############
        node.send_set_variables(motors(motors.left, motors.right))
        # return (motors.left, motors.right)
        #speed = -prox_front // 10
        #node.send_set_variables(motors(speed, speed))
        
    except KeyError:
        pass  # prox.horizontal not found

async def prog():
        node = await client.wait_for_node()
        await node.lock()
        await node.wait_for_variables({"prox.horizontal"})
        #--await node.watch(variables=True)
        #await obstacle_avoidance(node, variables)
        #(speedLeft, speedRight) = obstacle_avoidance(node, )
        node.add_variables_changed_listener(obstacle_avoidance)
        #    (motors.left, motors.right) = obstacle_avoidance(node, variables)
        #      await node.set_variables(motors.left, motors.right)
        #await node.set_variables(obstacle_avoidance(node, variables))
        await client.sleep()
        await node.set_variables(motors(0, 0))
        #await client.sleep(3)
        #await node.set_variables(motors(0, 0))
        await node.unlock()
        #await node.watch(variables=True)
        #node.add_variables_changed_listener(obstacle_avoidance)
        #await client.sleep()

client.run_async_program(prog)