import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_graph(x, x_RK4, t_s):
    fig_2d = plt.figure(figsize=(18, 10))
    fig_3d = plt.figure(figsize=(10,10))

    plot_3d_graph(fig_3d, x[0:3,:], x_RK4[0:3,:])

    # Grafico 2D spostamento X vs Y
    ax1 = fig_2d.add_subplot(3, 3, 1)
    ax1.plot(x[0,:], x[1, :], label='Spostamento X vs Y')
    ax1.plot(x_RK4[0,:], x_RK4[1, :], c='red')
    ax1.set_title("Evoluzione dello spostamento orizzontale")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.grid(True)

    # Grafico 2D altitudine
    ax2 = fig_2d.add_subplot(3, 3, 2)
    ax2.plot(t_s, x[2, :], label='Altitudine (z)')
    ax2.plot(t_s, x_RK4[2, :], c='red')
    ax2.set_title("Evoluzione dell\'altitudine del quadricottero")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("Altitudine (m)")
    ax2.grid(True)

    # Grafico 2D velocità X
    ax3 = fig_2d.add_subplot(3, 3, 3)
    ax3.plot(t_s, x[3, :], label='Velocità (x)')
    ax3.set_title("Evoluzione della velocità X del quadricottero")
    ax3.set_xlabel("Tempo (s)")
    ax3.set_ylabel("Velocity (m/s)")
    ax3.grid(True)

    # Grafico 2D velocità Y
    ax4 = fig_2d.add_subplot(3, 3, 4)
    ax4.plot(t_s, x[4, :], label='Velocità (y)')
    ax4.set_title("Evoluzione della velocità Y del quadricottero")
    ax4.set_xlabel("Tempo (s)")
    ax4.set_ylabel("Velocity (m/s)")
    ax4.grid(True)

    # Grafico 2D velocità Z
    ax5 = fig_2d.add_subplot(3, 3, 5)
    ax5.plot(t_s, x[5, :], label='Velocità (z)')
    ax5.set_title("Evoluzione della velocità Z del quadricottero")
    ax5.set_xlabel("Tempo (s)")
    ax5.set_ylabel("Velocity (m/s)")
    ax5.grid(True)

    # Grafico 2D roll
    ax6 = fig_2d.add_subplot(3, 3, 6)
    ax6.plot(t_s, x[6, :], label='Roll-phi (x)')
    ax6.set_title("Roll angle")
    ax6.set_xlabel("Tempo (s)")
    ax6.set_ylabel("Roll (deg)")
    ax6.grid(True)

    # Grafico 2D pitch
    ax7 = fig_2d.add_subplot(3, 3, 7)
    ax7.plot(t_s, x[7, :], label='Pitch-theta (y)')
    ax7.set_title("Pitch angle")
    ax7.set_xlabel("Tempo (s)")
    ax7.set_ylabel("Pitch (deg)")
    ax7.grid(True)

    # Grafico 2D yaw
    ax8 = fig_2d.add_subplot(3, 3, 8)
    ax8.plot(t_s, x[8, :], label='Yaw-psi (z)')
    ax8.set_title("yaw angle")
    ax8.set_xlabel("Tempo (s)")
    ax8.set_ylabel("Yaw (deg)")
    ax8.grid(True)


    # Regola gli spazi
    fig_2d.subplots_adjust(wspace=0.4, hspace=0.5)  # wspace = spazio tra colonne, hspace = spazio tra righe


    plt.show()


def plot_3d_graph(fig, pos, pos_RK4):
    ax = fig.add_subplot(1,1,1, projection='3d')

    ax.plot(pos[0,:], pos[1,:], pos[2,:], label='Traiettoria del drone')
    ax.plot(pos_RK4[0,:], pos_RK4[1,:], pos_RK4[2,:],c='red', label='Traiettoria del drone')
    #ax.scatter(x, y, z, c='red', marker='o')  # Opzionale: evidenzia i punti

    ax.set_xlabel('Asse X')
    ax.set_ylabel('Asse Y')
    ax.set_zlabel('Asse Z')
    ax.set_title('Posizione 3D del Drone')
    ax.grid(True)
