import numpy as np

class UAVState:
    def __init__(self):
        self.pos = np.zeros(3)     # x, y, z
        self.vel = np.zeros(3)
        self.mass = 1.5            # kg
