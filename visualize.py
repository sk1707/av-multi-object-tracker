import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

dt = 0.1
timesteps = 50

vehicle_truth = np.array([[10 + 2*i*dt, 5 + 0.5*i*dt] for i in range(timesteps)])
vehicle_meas  = vehicle_truth + np.random.normal(0, 0.3, vehicle_truth.shape)

ped_truth = np.array([[2 + 0.3*i*dt, 8 - 0.2*i*dt] for i in range(timesteps)])
ped_meas  = ped_truth + np.random.normal(0, 0.2, ped_truth.shape)

def run_ekf(measurements, init_state, q=0.1, r=0.3):
    states = []
    x = np.array(init_state, dtype=float)
    P = np.eye(4) * 10
    F = np.eye(4); F[0,2]=dt; F[1,3]=dt
    H = np.zeros((2,4)); H[0,0]=1; H[1,1]=1
    Q = np.eye(4) * q
    R = np.eye(2) * r
    for z in measurements:
        x = F @ x
        P = F @ P @ F.T + Q
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ (z - H @ x)
        P = (np.eye(4) - K @ H) @ P
        states.append(x[:2].copy())
    return np.array(states)

vehicle_est = run_ekf(vehicle_meas, [10, 5, 2, 0.5])
ped_est     = run_ekf(ped_meas,     [2, 8, 0.3, -0.2], q=0.05, r=0.2)

fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(0, 22); ax.set_ylim(3, 10)
ax.set_title("EKF Multi-Object Tracker — Urban Intersection", fontsize=14, fontweight='bold')
ax.set_xlabel("X Position (m)"); ax.set_ylabel("Y Position (m)")
ax.axhline(6.5, color='gray', linestyle='--', linewidth=0.8, label='Road boundary')
ax.axvline(11, color='gray', linestyle='--', linewidth=0.8)
ax.plot(vehicle_truth[:,0], vehicle_truth[:,1], 'b--', alpha=0.3, label='Vehicle truth')
ax.plot(ped_truth[:,0],     ped_truth[:,1],     'r--', alpha=0.3, label='Pedestrian truth')

v_meas_plot, = ax.plot([], [], 'b+', markersize=6, alpha=0.5, label='Vehicle LiDAR')
p_meas_plot, = ax.plot([], [], 'r+', markersize=6, alpha=0.5, label='Pedestrian LiDAR')
v_est_plot,  = ax.plot([], [], 'b-', linewidth=2,  label='Vehicle EKF estimate')
p_est_plot,  = ax.plot([], [], 'r-', linewidth=2,  label='Pedestrian EKF estimate')
v_dot,       = ax.plot([], [], 'bo', markersize=10)
p_dot,       = ax.plot([], [], 'ro', markersize=10)
time_text    = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=11)
ax.legend(loc='lower right', fontsize=9)

def animate(i):
    v_meas_plot.set_data(vehicle_meas[:i,0], vehicle_meas[:i,1])
    p_meas_plot.set_data(ped_meas[:i,0],     ped_meas[:i,1])
    v_est_plot.set_data(vehicle_est[:i,0],   vehicle_est[:i,1])
    p_est_plot.set_data(ped_est[:i,0],       ped_est[:i,1])
    if i > 0:
        v_dot.set_data([vehicle_est[i-1,0]], [vehicle_est[i-1,1]])
        p_dot.set_data([ped_est[i-1,0]],     [ped_est[i-1,1]])
    time_text.set_text(f't = {i*dt:.1f}s')
    return v_meas_plot, p_meas_plot, v_est_plot, p_est_plot, v_dot, p_dot, time_text

ani = animation.FuncAnimation(fig, animate, frames=timesteps, interval=100, blit=True)
ani.save('intersection_tracking.gif', writer='pillow', fps=10)
print("Saved intersection_tracking.gif")
plt.show()
