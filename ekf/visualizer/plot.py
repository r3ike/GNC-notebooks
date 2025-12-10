import numpy as np
import matplotlib.pyplot as plt

def debug_ekf_plots(
    t,
    nominal_pos, nominal_vel, nominal_euler,
    error_pos, error_vel, error_att, error_ba, error_bg,
    Pdiag,
    innov_gps_pos, innov_gps_vel, innov_mag, innov_baro,
    nees, nis_gps, nis_mag, nis_baro,
    imu_acc, imu_gyro,
    delay_latency, delay_index
):

    # ---------------------- 1. NOMINAL STATE -------------------------------
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8))
    axs[0].plot(t, nominal_pos)
    axs[0].set_title("Nominal Position")
    axs[0].legend(["x", "y", "z"])

    axs[1].plot(t, nominal_vel)
    axs[1].set_title("Nominal Velocity")
    axs[1].legend(["vx", "vy", "vz"])

    axs[2].plot(t, nominal_euler)
    axs[2].set_title("Nominal Euler Angles")
    axs[2].legend(["roll", "pitch", "yaw"])

    plt.tight_layout()

    # ---------------------- 2. ERROR STATE -------------------------------
    fig2, axs = plt.subplots(5, 1, figsize=(10, 12))
    axs[0].plot(t, error_pos)
    axs[0].set_title("Error Position (m)")
    axs[0].legend(["ex", "ey", "ez"])

    axs[1].plot(t, error_vel)
    axs[1].set_title("Error Velocity (m/s)")
    axs[1].legend(["evx", "evy", "evz"])

    axs[2].plot(t, error_att)
    axs[2].set_title("Error Orientation (rad)")
    axs[2].legend(["eroll", "epitch", "eyaw"])

    axs[3].plot(t, error_ba)
    axs[3].set_title("Accel Bias Error")
    axs[3].legend(["bax", "bay", "baz"])

    axs[4].plot(t, error_bg)
    axs[4].set_title("Gyro Bias Error")
    axs[4].legend(["bgx", "bgy", "bgz"])

    plt.tight_layout()

    # ---------------------- 3. INNOVATIONS -------------------------------
    fig3, axs = plt.subplots(4, 1, figsize=(10, 10))
    axs[0].plot(t, innov_gps_pos)
    axs[0].set_title("GPS Position Innovation")

    axs[1].plot(t, innov_gps_vel)
    axs[1].set_title("GPS Velocity Innovation")

    axs[2].plot(t, innov_mag)
    axs[2].set_title("Magnetometer Innovation (heading)")

    axs[3].plot(t, innov_baro)
    axs[3].set_title("Barometer Innovation (altitude)")

    plt.tight_layout()

    # ---------------------- 4. COVARIANCES -------------------------------
    fig4, axs = plt.subplots(3, 1, figsize=(10, 12))
    axs[0].plot(t, Pdiag[:, 0:3])
    axs[0].set_title("Covariance diag(P) - Position")

    axs[1].plot(t, Pdiag[:, 3:6])
    axs[1].set_title("Covariance diag(P) - Velocity")

    axs[2].plot(t, Pdiag[:, 6:9])
    axs[2].set_title("Covariance diag(P) - Orientation")

    plt.tight_layout()

    # ---------------------- 5. DELAYED STATE DEBUG -------------------------------
    fig5, axs = plt.subplots(2, 1, figsize=(10, 6))
    axs[0].plot(t, delay_latency)
    axs[0].set_title("Estimated Sensor Latency (s)")

    axs[1].plot(t, delay_index)
    axs[1].set_title("Buffer Index Used for Replay")
    plt.tight_layout()

    # ---------------------- 6. CONSISTENCY -------------------------------
    fig6, axs = plt.subplots(3, 1, figsize=(10, 8))
    axs[0].plot(t, nees)
    axs[0].set_title("NEES")

    axs[1].plot(t, nis_gps)
    axs[1].set_title("NIS - GPS")

    axs[2].plot(t, nis_mag)
    axs[2].set_title("NIS - Magnetometer")

    plt.tight_layout()

    # ---------------------- 7. RAW SENSOR DATA -------------------------------
    fig7, axs = plt.subplots(2, 1, figsize=(10, 6))
    axs[0].plot(t, imu_acc)
    axs[0].set_title("Accelerometer Raw")

    axs[1].plot(t, imu_gyro)
    axs[1].set_title("Gyroscope Raw")

    plt.tight_layout()
    plt.show()
