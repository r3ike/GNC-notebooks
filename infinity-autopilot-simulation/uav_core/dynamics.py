import numpy as np

def dynamics(state, thrust, dt=0.01):
    g = np.array([0, 0, -9.81])
    acc = np.array([0, 0, thrust / state.mass]) + g
    state.vel += acc * dt
    state.pos += state.vel * dt
