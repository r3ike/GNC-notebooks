import numpy as np

def quat_norm(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)  # normalizzazione

def rotmat_body_to_ned(q):

    qw, qx, qy, qz = quat_norm(q)
    
    R = np.array([
        [qw*qw + qx*qx - qy*qy - qz*qz,     2*(qx*qy + qw*qz),           2*(qx*qz - qw*qy)],
        [2*(qx*qy - qw*qz),                 qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz + qw*qx)],
        [2*(qx*qz + qw*qy),                 2*(qy*qz - qw*qx),           qw*qw - qx*qx - qy*qy + qz*qz]
    ])
    
    return R

def quat_multiply(q1, q2):
    """
    Moltiplicazione tra quaternioni (scalar-first):
    q = [qw, qx, qy, qz].
    Restituisce q1 ⊗ q2.
    """
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return np.array([w, x, y, z])

def quat_update(q, delta_theta):
    theta = np.linalg.norm(delta_theta)

    u = delta_theta / theta

    delta_q = np.array([np.cos(theta/2), 
                        u[0] * np.sin(theta / 2), 
                        u[1] * np.sin(theta / 2), 
                        u[2] * np.sin(theta / 2)
                        ]).transpose()
    
    q_new = quat_multiply(delta_q, q)

    return quat_norm(q_new)

def skew(v):
    return np.array([
        [0, -v[2],  v[1]],
        [v[2],  0, -v[0]],
        [-v[1], v[0],  0]
    ])