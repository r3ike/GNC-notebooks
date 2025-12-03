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

# helper: crea quaternion da small-angle (3,) -> quat ~ [1, 1/2 dtheta]
def quat_from_small_angle(dtheta):
    dq = np.zeros(4)
    dq[0] = 1.0
    dq[1:4] = 0.5 * dtheta
    # normalizzazione opzionale (per piccoli angoli è ok)
    return dq

def skew(v):
    return np.array([
        [0, -v[2],  v[1]],
        [v[2],  0, -v[0]],
        [-v[1], v[0],  0]
    ])


# Costanti ellissoide WGS84
a = 6378137.0            # semiasse maggiore (m)
f = 1 / 298.257223563    # appiattimento
b = a * (1 - f)           # semiasse minore
e2 = 1 - (b**2 / a**2)    # eccentricità al quadrato

def gps_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """
    Converte coordinate GPS (lat, lon, alt) in spostamento NED
    rispetto a un punto di riferimento.
    
    Input in gradi e metri.
    Output: np.array([North, East, Down]) in metri
    """

    # converti in radianti
    lat  = np.radians(lat)
    lon  = np.radians(lon)
    lat0 = np.radians(lat0)
    lon0 = np.radians(lon0)

    # -----------------------
    # 1) Converti in ECEF
    # -----------------------
    def lla_to_ecef(lat, lon, alt):
        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        x = (N + alt) * np.cos(lat) * np.cos(lon)
        y = (N + alt) * np.cos(lat) * np.sin(lon)
        z = (N*(1 - e2) + alt) * np.sin(lat)
        return np.array([x, y, z])

    ecef    = lla_to_ecef(lat, lon, alt)
    ecef0   = lla_to_ecef(lat0, lon0, alt0)
    de      = ecef - ecef0  # vettore differenza in ECEF

    # -----------------------
    # 2) Matrice di rotazione ECEF → NED
    # -----------------------
    R = np.array([
        [-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0),  np.cos(lat0)],
        [       -np.sin(lon0),               np.cos(lon0),           0       ],
        [-np.cos(lat0)*np.cos(lon0), -np.cos(lat0)*np.sin(lon0), -np.sin(lat0)]
    ])

    ned = R @ de  # spostamento NED in metri
    return ned
