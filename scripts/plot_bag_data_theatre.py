#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

plot_dir = '/home/upo/marsupial/src/marsupial_simulator_ros2/plots/plots_theatre'
os.makedirs(plot_dir, exist_ok=True)

drone_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/uav_data_theatre.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data_theatre.csv')

ugv_data_test = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')
drone_data_test = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/drone_data.csv')

drone_time = drone_data['time'].to_numpy()
drone_position_x = drone_data['imu_orientation_x'].to_numpy() 
drone_position_y = drone_data['imu_orientation_y'].to_numpy()  
drone_position_z = drone_data['imu_orientation_z'].to_numpy()

ugv_time = ugv_data['time'].to_numpy()
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()

time_scale = 6.25

ugv_time_test = ugv_data_test['time'].to_numpy() / time_scale
ugv_position_x_test = ugv_data_test['position_x'].to_numpy()
ugv_position_y_test = ugv_data_test['position_y'].to_numpy()
ugv_position_z_test = ugv_data_test['position_z'].to_numpy()

drone_time_test = drone_data_test['time'].to_numpy() / time_scale
drone_position_x_test = drone_data_test['position_x'].to_numpy()
drone_position_y_test = drone_data_test['position_y'].to_numpy()
drone_position_z_test = drone_data_test['position_z'].to_numpy()

# Graficar posición en 3D del UGV y del UAV
def plot_3d(ugv_position_x, ugv_position_y, ugv_position_z, drone_position_x, drone_position_y, drone_position_z, step=100):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(ugv_position_x, ugv_position_y, ugv_position_z, label='UGV Position', color='b')
    ax.plot(drone_position_x, drone_position_y, drone_position_z, label='UAV Position', color='r')

    # Descomentar para añadir líneas que conectan las posiciones del UGV y el UAV cada cierto paso
    # num_points = min(len(ugv_position_x), len(drone_position_x))
    # for i in range(0, num_points, step):
    #     x = [ugv_position_x[i], drone_position_x[i]]
    #     y = [ugv_position_y[i], drone_position_y[i]]
    #     z = [ugv_position_z[i], drone_position_z[i]]
    #     ax.plot(x, y, z, color='gray')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Position of UGV and UAV')
    ax.legend()
    ax.grid(True)
    
    set_axes_equal(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position_theatre.png'))
    plt.show()
    plt.close()

def set_axes_equal(ax):
    """Función para ajustar las escalas del gráfico 3D."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    
    max_range = max([x_range, y_range, z_range])

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim([z_middle - max_range / 2, z_middle + max_range / 2])

# Graficar comparación 3D del UGV y UAV entre el teatro y el test
def plot_3d_comparative(ugv_position_x_theatre, ugv_position_y_theatre, ugv_position_z_theatre,
                        ugv_position_x_test, ugv_position_y_test, ugv_position_z_test,
                        drone_position_x_theatre, drone_position_y_theatre, drone_position_z_theatre,
                        drone_position_x_test, drone_position_y_test, drone_position_z_test, step=100):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Trayectoria del UGV y UAV en el teatro
    ax.plot(ugv_position_x_theatre, ugv_position_y_theatre, ugv_position_z_theatre, label='UGV Position (Theatre)', color='b')
    ax.plot(drone_position_x_theatre, drone_position_y_theatre, drone_position_z_theatre, label='UAV Position (Theatre)', color='r')

    # Trayectoria del UGV y UAV en el test anterior
    ax.plot(ugv_position_x_test, ugv_position_y_test, ugv_position_z_test, label='UGV Position (Test)', color='b', linestyle='--')
    ax.plot(drone_position_x_test, drone_position_y_test, drone_position_z_test, label='UAV Position (Test)', color='r', linestyle='--')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Position Comparison: UGV and UAV Theatre vs Test')
    ax.legend()
    ax.grid(True)

    set_axes_equal(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position_comparative_ugv_uav.png'))
    plt.show()
    plt.close()

plt.figure(figsize=(10, 8))

# Subplot 1: Comparativa de la posición en el eje X
plt.subplot(3, 1, 1)
plt.plot(drone_time, drone_position_x, label='Position X (Theatre)', color='b')
plt.plot(drone_time_test, drone_position_x_test, label='Position X (Test)', color='b', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('UAV Position Comparison - X')
plt.legend()
plt.grid(True)

# Subplot 2: Comparativa de la posición en el eje Y
plt.subplot(3, 1, 2)
plt.plot(drone_time, drone_position_y, label='Position Y (Theatre)', color='g')
plt.plot(drone_time_test, drone_position_y_test, label='Position Y (Test)', color='g', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('UAV Position Comparison - Y')
plt.legend()
plt.grid(True)

# Subplot 3: Comparativa de la posición en el eje Z
plt.subplot(3, 1, 3)
plt.plot(drone_time, drone_position_z, label='Position Z (Theatre)', color='r')
plt.plot(drone_time_test, drone_position_z_test, label='Position Z (Test)', color='r', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Z (m)')
plt.title('UAV Position Comparison - Z')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'uav_position_comparative_theatre_test.png'))
plt.close()

plt.figure(figsize=(10, 8))

# Subplot 1: Comparativa de la posición en el eje X
plt.subplot(2, 1, 1)
plt.plot(ugv_time, ugv_position_x, label='Position X (Theatre)', color='b')
plt.plot(ugv_time_test, ugv_position_x_test, label='Position X (Test)', color='b', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('UGV Position Comparison - X')
plt.legend()
plt.grid(True)

# Subplot 2: Comparativa de la posición en el eje Y
plt.subplot(2, 1, 2)
plt.plot(ugv_time, ugv_position_y, label='Position Y (Theatre)', color='g')
plt.plot(ugv_time_test, ugv_position_y_test, label='Position Y (Test)', color='g', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('UGV Position Comparison - Y')
plt.legend()
plt.grid(True)

# Subplot 3: Comparativa de la posición en el eje Z
# plt.subplot(3, 1, 3)
# plt.plot(ugv_time, ugv_position_z, label='Position Z (Theatre)', color='r')
# plt.plot(ugv_time_test, ugv_position_z_test, label='Position Z (Test)', color='r', linestyle='--')
# plt.xlabel('Time (s)')
# plt.ylabel('Position Z (m)')
# plt.title('UGV Position Comparison - Z')
# plt.legend()
# plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'ugv_position_comparative_theatre_test.png'))
plt.close()

# Comparativa de posiciones del UGV y UAV en 3D (Theatre vs Test)
plot_3d_comparative(ugv_position_x, ugv_position_y, ugv_position_z, ugv_position_x_test, ugv_position_y_test, ugv_position_z_test,
                    drone_position_x, drone_position_y, drone_position_z, drone_position_x_test, drone_position_y_test, drone_position_z_test)

plot_3d(ugv_position_x, ugv_position_y, ugv_position_z, drone_position_x, drone_position_y, drone_position_z)
