import quadcopter
import numerical_integrator
import numpy as np
from visualizer.plot import plot_graph


x_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).transpose()

tf_s = 10 # t final step in second
dt = 0.004 # time step

t_s = np.arange(0, tf_s, dt)    #time step
nt_s = t_s.size                 #number of timestep

x = np.empty((12, nt_s), dtype=float)
#x_euler = np.empty((12, nt_s), dtype=float)

x[:,0] = x_0
#x_euler[:,0] = x_0

u = np.array([4, 4, 4, 4]).transpose()

#run simulation
for i in range(1, nt_s):
    x[:,i] = numerical_integrator.trapeziodal(quadcopter.quadcopter_dynamics , x[:,i-1], u , dt)
    #x_euler[:,i] = numerical_integrator.forward_euler(quadcopter.quadcopter_dynamics , x_euler[:,i-1], u , dt)


#salvataggio e visualizzazione dei dati
header = "x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r"
with open("./data/state.csv", "w", newline="\n") as f:
    np.savetxt(f, x.T, delimiter=",",header=header, fmt="%.5f")

plot_graph(x, t_s)