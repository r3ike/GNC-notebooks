import numpy as np
import math

def quat_to_euler(q):
    """
    Converte un quaternione [qw, qx, qy, qz] in angoli di Eulero (roll, pitch, yaw)
    nel convention roll->pitch->yaw (ZYX).
    Ritorna gli angoli in radianti.
    """
    q = np.asarray(q, dtype=float)
    
    # normalizza per sicurezza
    qw, qx, qy, qz = quat_norm(q)

    # Roll (x-axis rotation)
    sinr = 2 * (qw*qx + qy*qz)
    cosr = 1 - 2 * (qx*qx + qy*qy)
    roll = np.arctan2(sinr, cosr)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw*qy - qz*qx)
    sinp = np.clip(sinp, -1.0, 1.0)  # evita errori numerici
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny = 2 * (qw*qz + qx*qy)
    cosy = 1 - 2 * (qy*qy + qz*qz)
    yaw = np.arctan2(siny, cosy)

    return np.array([roll, pitch, yaw])

def quat_norm(q):
    q = np.asarray(q, dtype=float)

    if np.linalg.norm(q) < 1e-12:
        q = np.array([1,0,0,0])
    else:
        q = q / np.linalg.norm(q)
    return q

def safe_rotmat_body_to_ned(q):
    """Wrapper sicuro per la matrice di rotazione"""
    q = np.asarray(q).ravel()
    
    # Normalizza
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        print("WARNING: Zero quaternion, using identity")
        return np.eye(3)
    
    q_normalized = q / norm
    
    # Controlla NaN
    if np.any(np.isnan(q_normalized)):
        print("WARNING: NaN in quaternion, using identity")
        return np.eye(3)
    
    try:
        return rotmat_body_to_ned(q_normalized)
    except:
        print("ERROR: Rotation matrix computation failed")
        return np.eye(3)
    
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




def integrate_quaternion_px4( q, gyro, dt):
    """
    Integrazione del quaternione identica a PX4
    
    Args:
        q: Quaternione corrente
        gyro: Misura del giroscopio [rad/s] (3,)
        dt: Timestep [s]
        
    Returns:
        Quaternione aggiornato
    """
    # Calcola l'angolo di rotazione integrato
    delta_angle = gyro * dt
    
    # Norma dell'angolo di rotazione
    delta_angle_norm = np.linalg.norm(delta_angle)
    
    # Quaternione incrementale
    dq = np.array([1,0,0,0], dtype= np.float64)
    
    if delta_angle_norm > 1e-12:
        # Formula esatta per rotazioni finite (identica a PX4)
        theta = delta_angle_norm * 0.5
        theta_sq = theta * theta
        
        # Approssimazione sin(x)/x per x piccolo (identica a PX4)
        if theta_sq < 1.0e-4:
            # Serie di Taylor: sin(theta)/theta ≈ 1 - theta²/6
            sin_theta_over_theta = 1.0 - theta_sq / 6.0
        else:
            sin_theta_over_theta = math.sin(theta) / theta
        
        cos_theta = math.cos(theta)
        
        # Quaternione di rotazione incrementale (identico a PX4)
        dq[0] = cos_theta
        dq[1] = delta_angle[0] * 0.5 * sin_theta_over_theta
        dq[2] = delta_angle[1] * 0.5 * sin_theta_over_theta
        dq[3] = delta_angle[2] * 0.5 * sin_theta_over_theta
    else:
        # Per rotazioni molto piccole (identico a PX4)
        dq[0] = 1.0
        dq[1] = delta_angle[0] * 0.5
        dq[2] = delta_angle[1] * 0.5
        dq[3] = delta_angle[2] * 0.5
        
        # Normalizza (come in PX4)
        dq_norm = dq.norm()
        if dq_norm > 1e-12:
            dq.data /= dq_norm
    
    # Aggiorna il quaternione: q_new = q_old ⊗ dq
    return quat_multiply(q, dq)
    
def integrate_quaternion_optimized(q, gyro, dt):
    """
    Versione ottimizzata identica a quella in ecl/AttitudeEstimator.cpp
    """
    # Limita la velocità angolare massima (come in PX4)
    gyro_norm = np.linalg.norm(gyro)
    #if gyro_norm > self.MAX_ANGULAR_RATE:
    #    gyro = gyro * (self.MAX_ANGULAR_RATE / gyro_norm)
    
    # Calcola l'angolo di rotazione
    delta_angle = gyro * dt
    angle_sq = np.sum(delta_angle**2)
    angle = math.sqrt(angle_sq)
    
    dq = np.array([1,0,0,0],dtype= np.float64)
    
    if angle < 1e-6:
        # Approssimazione del primo ordine per angoli piccoli
        dq[0] = 1.0
        dq[1] = 0.5 * delta_angle[0]
        dq[2] = 0.5 * delta_angle[1]
        dq[3] = 0.5 * delta_angle[2]
    else:
        # Formula esatta (identica a PX4)
        sin_half_angle = math.sin(0.5 * angle)
        cos_half_angle = math.cos(0.5 * angle)
        scale = sin_half_angle / angle
        
        dq[0] = cos_half_angle
        dq[1] = delta_angle[0] * scale
        dq[2] = delta_angle[1] * scale
        dq[3] = delta_angle[2] * scale
    
    # Moltiplica i quaternioni: q_new = q_old ⊗ dq
    return quat_multiply(q, dq)