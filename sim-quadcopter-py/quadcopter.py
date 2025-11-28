import numpy as np
from sim_tools import rotation_matrix_body_to_inertial, euler_angle_rates

#      0  1  2    3      4      5     6     7     8   9  10 11
# x = [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r]

# x_dot = [x_dot, y_dot, z_dot, acc_x, acc_y, acc_z, phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]

# u = [T_1, T_2, T_3, T_4]

# Costanti del quadricottero
m = 1.2       # Massa (kg)
g = 9.81      # Accelerazione di gravità (m/s^2)
l = 0.25      # Distanza dal centro ai rotori (m)
Ixx = 0.02    # Momento d'inerzia sull'asse x (kg·m²)
Iyy = 0.02    # Momento d'inerzia sull'asse y (kg·m²)
Izz = 0.04    # Momento d'inerzia sull'asse z (kg·m²)

rho = 1.225      # Densità dell'aria (kg/m^3)
Cd = 1.0         # Coefficiente di resistenza aerodinamica
A = 0         # Area frontale approssimativa (m^2)

Jr = 0.01    # Momento d'inerzia dei rotori (kg·m²)
I = np.diag([Ixx, Iyy, Izz])

def quadcopter_dynamics(x, u):
    
    thrust, tau = calculate_momentum_trust(u)   #calcolo i momenti angolari


    R  = rotation_matrix_body_to_inertial(x[6],x[7],x[8])           #precompilo la matrice di rotazione da body a inertial frame

    
    accel = (R @ np.array([0,0,thrust]) + drag_force(np.array([x[3], x[4], x[5]])) + wind_force() - np.array([0,0,m * g]))/m       #calcolo accelerazione 

    
    omega = np.array([x[9], x[10], x[11]])                              #estrapolo le velocità angolari dal vettore degli stati

    motor_gyro_torque = gyro_torque(estimate_motor_angular_velocity(u),omega)

    ang_accel = np.linalg.inv(I) @ (tau + motor_gyro_torque - np.cross(omega, I @ omega))   #calcolo la nuova accelerazione angolare

    T = euler_angle_rates(x[6],x[7])        #precompilo la matrice per convertire da body rotation rate a euler rate
    euler_rate = T @ omega                  #calcolo gli euler rate

    return np.array([x[3], x[4], x[5], accel[0], accel[1], accel[2], euler_rate[0], euler_rate[1], euler_rate[2], ang_accel[0], ang_accel[1], ang_accel[2]]).transpose()


# calcolo dei momenti angolari, della spinta totale
def calculate_momentum_trust(u):
    thrust = sum(u)

    tau_phi = l * (u[1] - u[3])
    tau_theta = l * (u[0] - u[2])
    tau_psi = 0.1 * (u[0] - u[1] + u[2] - u[3])
    tau = np.array([tau_phi, tau_theta, tau_psi])

    return thrust, tau


# Forza di resistenza aerodinamica
def drag_force(velocity):
    v_norm = np.linalg.norm(velocity)
    return -0.5 * rho * Cd * A * velocity * v_norm


def estimate_motor_angular_velocity(u):
    omega_rotors = np.sqrt(np.abs(u))  # Stima della velocità angolare dei rotori
    return omega_rotors

#effetto giroscopico dei motori
def gyro_torque(omega_rotors, omega_quadcopter):
    gyro_torque = np.array([0, 
                            Jr * (omega_rotors[0] - omega_rotors[1] + omega_rotors[2] - omega_rotors[3]) * omega_quadcopter[0],
                            Jr * (omega_rotors[0] - omega_rotors[1] + omega_rotors[2] - omega_rotors[3]) * omega_quadcopter[1]
                            ])
    
    return gyro_torque

def wind_force():
    turbulence = np.random.normal(0, 0.2, size=3)  # Turbolenze casuali
    return turbulence