import numpy as np
import matplotlib.pyplot as plt

# Parametri
dt = 0.01  # Passo temporale
t_end = 5  # Tempo finale
time = np.arange(0, t_end, dt)

# Funzione velocità senza rumore: v(t) = 3t^2 + 2t + 1
def velocity(t):
    return 3 * t**2 + 2 * t + 1

# Genera rumore gaussiano con deviazione standard
def add_noise(value, std_dev=1):
    return value + np.random.normal(0, std_dev)

# Integrazione di Eulero con rumore
pos_euler = [0]
for t in time[:-1]:
    v = add_noise(velocity(t))
    new_pos = pos_euler[-1] + v * dt
    pos_euler.append(new_pos)

# Integrazione Trapezoidale con rumore
pos_trapezoidal = [0]
prev_v = add_noise(velocity(0))
for t in time[:-1]:
    v = add_noise(velocity(t + dt))
    new_pos = pos_trapezoidal[-1] + 0.5 * (prev_v + v) * dt
    pos_trapezoidal.append(new_pos)
    prev_v = v

# Soluzione analitica: posizione integrata analiticamente
pos_analytical = time**3 + time**2 + time

# Grafico
plt.figure(figsize=(8, 5))
plt.plot(time, pos_analytical, label="Analitica", linestyle='--', color='black')
plt.plot(time, pos_euler, label="Eulero (con rumore)", linestyle='-', color='red', alpha=0.7)
plt.plot(time, pos_trapezoidal, label="Trapezoidale (con rumore)", linestyle='-', color='blue', alpha=0.7)
plt.xlabel("Tempo (s)")
plt.ylabel("Posizione (m)")
plt.title("Confronto Integrazione con Rumore: Eulero vs Trapezoidale")
plt.legend()
plt.grid(True)
plt.show()
