import numpy as np

# Matrice di rotazione
def rotation_matrix_body_to_inertial(phi, theta, psi):
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                  [np.sin(psi),  np.cos(psi), 0],
                  [0, 0, 1]])
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                   [0, 1, 0],
                   [-np.sin(theta), 0, np.cos(theta)]])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi), np.cos(phi)]])
    return Rz @ Ry @ Rx


# Matrice di trasformazione da rate angolari a derivata degli angoli di Eulero
def euler_angle_rates(phi, theta):
    return np.array([
        [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
    ])