from state import UAVState
from dynamics import dynamics

state = UAVState()

def step(thrust, dt=0.01):
    dynamics(state, thrust, dt)
    return state
