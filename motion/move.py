from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

client = ClientAsync()

async def prog():
    node = await client.wait_for_node()
    await node.lock()
    await node.set_variables(motors(50, 50))
    await client.sleep(2)
    await node.set_variables(motors(0, 0))
    await node.unlock()

client.run_async_program(prog)
