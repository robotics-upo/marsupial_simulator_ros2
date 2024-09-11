#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from mpl_toolkits.mplot3d import Axes3D

plot_dir = '/home/upo/marsupial/src/marsupial_simulator_ros2/plots'
os.makedirs(plot_dir, exist_ok=True)

drone_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/drone_data.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')

time_scale = 6.25

ugv_time = ugv_data['time'].to_numpy() / time_scale
ugv_cable_length = ugv_data['cable_length'].to_numpy()
ugv_target_length = ugv_data['target_length'].to_numpy()
ugv_uav_distance = ugv_data['distance'].to_numpy()
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()
ugv_velocity_x = ugv_data['velocity_x'].to_numpy()
ugv_velocity_y = ugv_data['velocity_y'].to_numpy()
ugv_velocity_z = ugv_data['velocity_z'].to_numpy()
ugv_target_x = ugv_data['target_position_x'].to_numpy()
ugv_target_y = ugv_data['target_position_y'].to_numpy()
ugv_target_z = ugv_data['target_position_z'].to_numpy()

drone_time = drone_data['time'].to_numpy() / time_scale
drone_position_x = drone_data['position_x'].to_numpy()
drone_position_y = drone_data['position_y'].to_numpy()
drone_position_z = drone_data['position_z'].to_numpy()
drone_velocity_x = drone_data['velocity_x'].to_numpy()
drone_velocity_y = drone_data['velocity_y'].to_numpy()
drone_velocity_z = drone_data['velocity_z'].to_numpy()
drone_target_x = drone_data['target_position_x'].to_numpy()
drone_target_y = drone_data['target_position_y'].to_numpy()
drone_target_z = drone_data['target_position_z'].to_numpy()

# Graficar tether_distance
plt.figure(figsize=(10, 8))
plt.plot(ugv_time, ugv_cable_length, label='Cable Length')
plt.plot(ugv_time, ugv_target_length, 'r--', label='Target Length')  
plt.plot(ugv_time, ugv_uav_distance, label='Distance')  
# plt.plot(ugv_time, (ugv_target_length - 0.3)/1.0, label='Distance')  
plt.xlabel('Time (s)')
plt.ylabel('Length (m)')
plt.title('Tether Distance')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'tether_distance.png'))
plt.close()

# Graficar tether_distance_filtered
start_time = 120
end_time = 130
ugv_time_filtered = ugv_time[(ugv_time >= start_time) & (ugv_time <= end_time)]
ugv_cable_length_filtered = ugv_cable_length[(ugv_time >= start_time) & (ugv_time <= end_time)]
ugv_target_length_filtered = ugv_target_length[(ugv_time >= start_time) & (ugv_time <= end_time)]
ugv_uav_distance_filtered = ugv_uav_distance[(ugv_time >= start_time) & (ugv_time <= end_time)]
plt.figure(figsize=(10, 8))
plt.plot(ugv_time_filtered, ugv_cable_length_filtered, label='Cable Length')
plt.plot(ugv_time_filtered, ugv_target_length_filtered, 'r--', label='Target Length')  
plt.plot(ugv_time_filtered, ugv_uav_distance_filtered, label='Distance')  
plt.xlabel('Time (s)')
plt.ylabel('Length (m)')
plt.title(f'Tether Distance (Time {start_time} - {end_time} s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'tether_distance_filtered.png'))
plt.close()

# Graficar drone_position
plt.figure(figsize=(10, 8))

# Subplot 1: Posición en el eje X
plt.subplot(3, 1, 1)
plt.plot(drone_time, drone_position_x, label='Position X')
plt.plot(drone_time, drone_target_x, 'r-', label='Target X')
# plt.plot(drone_time, drone_target_x + 0.2, 'orange', linestyle='--', label='Max. tolerance')
# plt.plot(drone_time, drone_target_x - 0.2, 'orange', linestyle='--', label='Min. tolerance')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('Drone Position - X')
plt.legend()
plt.grid(True)

# Subplot 2: Posición en el eje Y
plt.subplot(3, 1, 2)
plt.plot(drone_time, drone_position_y, label='Position Y')
plt.plot(drone_time, drone_target_y, 'r-', label='Target Y')
# plt.plot(drone_time, drone_target_y + 0.2, 'orange', linestyle='--', label='Max. tolerance')
# plt.plot(drone_time, drone_target_y - 0.2, 'orange', linestyle='--', label='Min. tolerance')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('Drone Position - Y')
plt.legend()
plt.grid(True)

# Subplot 3: Posición en el eje Z
plt.subplot(3, 1, 3)
plt.plot(drone_time, drone_position_z, label='Position Z')
plt.plot(drone_time, drone_target_z, 'r-', label='Target Z')
# plt.plot(drone_time, drone_target_z + 0.2, 'orange', linestyle='--', label='Max. tolerance')
# plt.plot(drone_time, drone_target_z - 0.2, 'orange', linestyle='--', label='Min. tolerance')
plt.xlabel('Time (s)')
plt.ylabel('Position Z (m)')
plt.title('Drone Position - Z')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'drone_position.png'))
plt.close()

# Graficar drone_velocity
plt.figure(figsize=(10, 4))

drone_velocity = np.sqrt(drone_velocity_x**2 + drone_velocity_y**2 + drone_velocity_z**2)
plt.plot(drone_time, drone_velocity, label='Drone Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Drone Velocity')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'drone_velocity.png'))
plt.close()

# Graficar UGV_position
plt.figure(figsize=(10, 8))

# Subplot 1: Posición en el eje X
plt.subplot(2, 1, 1)
plt.plot(ugv_time, ugv_position_x, label='Position X')
plt.plot(ugv_time, ugv_target_x, 'r-', label='Target X')
# plt.plot(ugv_time, ugv_target_x + 0.1, 'orange', linestyle='--', label='Max. tolerance')
# plt.plot(ugv_time, ugv_target_x - 0.1, 'orange', linestyle='--', label='Min. tolerance')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('UGV Position - X')
plt.legend()
plt.grid(True)

# Subplot 2: Posición en el eje Y
plt.subplot(2, 1, 2)
plt.plot(ugv_time, ugv_position_y, label='Position Y')
plt.plot(ugv_time, ugv_target_y, 'r-', label='Target Y')
# plt.plot(ugv_time, ugv_target_y + 0.1, 'orange', linestyle='--', label='Max. tolerance')
# plt.plot(ugv_time, ugv_target_y - 0.1, 'orange', linestyle='--', label='Min. tolerance')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('UGV Position - Y')
plt.legend()
plt.grid(True)

# Subplot 3: Posición en el eje Z
# plt.subplot(3, 1, 3)
# plt.plot(ugv_time, ugv_position_z, label='Position Z')
# plt.plot(ugv_time, ugv_target_z, 'r-', label='Target Z')
# plt.plot(ugv_time, ugv_target_z + 0.2, 'orange', linestyle='--', label='Target Z + 0.2')
# plt.plot(ugv_time, ugv_target_z - 0.2, 'orange', linestyle='--', label='Target Z - 0.2')
# plt.xlabel('Time (s)')
# plt.ylabel('Position Z (m)')
# plt.title('UGV Position - Z')
# plt.legend()
# plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'ugv_position.png'))
plt.close()

# Graficar UGV_velocity
plt.figure(figsize=(10, 4))

ugv_velocity = np.sqrt(ugv_velocity_x**2 + ugv_velocity_y**2 + ugv_velocity_z**2)
plt.plot(ugv_time, ugv_velocity, label='UGV Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('UGV Velocity')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'ugv_velocity.png'))
plt.close()

def catenary(x1, y1, z1, x2, y2, z2, n_points=100):
    """Calcula los puntos de una parábola que une (x1, y1, z1) y (x2, y2, z2)"""
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    d = np.sqrt(dx**2 + dy**2)
    
    x = np.linspace(0, d, n_points)
    a = dz / (d**2)
    z = a * (x - d/2)**2 + z1

    x_rot = x1 + (x * (dx / d))
    y_rot = y1 + (x * (dy / d))

    return x_rot, y_rot, z

def set_axes_equal(ax):
    """Set 3D plot axes to equal scale."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# Graficar posición 3D del UGV y del dron
def plot_3d(ugv_position_x, ugv_position_y, ugv_position_z, drone_position_x, drone_position_y, drone_position_z, step, use_catenary):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(ugv_position_x, ugv_position_y, ugv_position_z, label='UGV Position')
    ax.plot(drone_position_x, drone_position_y, drone_position_z, label='Drone Position')

    num_points = min(len(ugv_position_x), len(drone_position_x))
    for i in range(0, num_points, step):
        if use_catenary:
            x, y, z = catenary(ugv_position_x[i], ugv_position_y[i], ugv_position_z[i], 
                               drone_position_x[i], drone_position_y[i], drone_position_z[i])
        else:
            x = [ugv_position_x[i], drone_position_x[i]]
            y = [ugv_position_y[i], drone_position_y[i]]
            z = [ugv_position_z[i], drone_position_z[i]]
        color = plt.cm.plasma(i / num_points)  
        # ax.plot(x, y, z, color=color)
        ax.scatter(x[0], y[0], z[0], color='blue', s=5) 
        ax.scatter(x[-1], y[-1], z[-1], color=color, s=5)   

    ax.set_xlabel('Position X (m)')
    ax.set_ylabel('Position Y (m)')
    ax.set_zlabel('Position Z (m)')
    ax.set_title('3D Position of UGV and Drone')
    ax.legend()
    ax.grid(True)
    set_axes_equal(ax)
    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position.png'))

    plt.show(block=True)
    plt.close()

plot_3d(ugv_position_x, ugv_position_y, ugv_position_z, drone_position_x, drone_position_y, drone_position_z, step=2000, use_catenary=False)  



# Simulation time (s)
simulation_time = ugv_time[-1] - ugv_time[0]
print(f"Simulation time: {simulation_time:.2f} s")

# Target numbers: Number of target points (-)
target_changes = (np.diff(drone_target_x) != 0) | (np.diff(drone_target_y) != 0) | (np.diff(drone_target_z) != 0)
target_numbers = np.sum(target_changes) - 1 
print(f"Target numbers: {target_numbers} points")

# Distance covered UGV: Distance travelled by UGV (m)
ugv_distances = np.sqrt(np.diff(ugv_position_x)**2 + np.diff(ugv_position_y)**2 + np.diff(ugv_position_z)**2)
distance_covered_ugv = np.sum(ugv_distances)
print(f"Distance covered by UGV: {distance_covered_ugv:.2f} m")

# Distance covered UAV: Distance travelled by UAV (m)
uav_distances = np.sqrt(np.diff(drone_position_x)**2 + np.diff(drone_position_y)**2 + np.diff(drone_position_z)**2)
distance_covered_uav = np.sum(uav_distances)
print(f"Distance covered by UAV: {distance_covered_uav:.2f} m")

# Total tether released: Length of tether released (m)
tether_released = np.sum(np.diff(ugv_cable_length)[np.diff(ugv_cable_length) > 0])
print(f"Total tether released: {tether_released:.2f} m")

# Total tether collected: Length of tether collected (m)
tether_collected = np.sum(np.abs(np.diff(ugv_cable_length)[np.diff(ugv_cable_length) < 0]))
print(f"Total tether collected: {tether_collected:.2f} m")
