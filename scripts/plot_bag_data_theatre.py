#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Directory to save the plots
plot_dir = '/home/upo/marsupial/src/marsupial_simulator_ros2/plots/plots_theatre'
os.makedirs(plot_dir, exist_ok=True)

# Load data from CSV files
# Theatre data
drone_data_theatre = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/uav_data_theatre.csv')
ugv_data_theatre = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data_theatre.csv')

# Test data
drone_data_test = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/drone_data.csv')
ugv_data_test = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')

# Extract UAV data from theatre experiment
drone_time_theatre = drone_data_theatre['time'].to_numpy()
drone_position_x_theatre = drone_data_theatre['position_x'].to_numpy()
drone_position_y_theatre = drone_data_theatre['position_y'].to_numpy()
drone_position_z_theatre = drone_data_theatre['position_z'].to_numpy()

# Extract UGV data from theatre experiment
ugv_time_theatre = ugv_data_theatre['time'].to_numpy()
ugv_position_x_theatre = ugv_data_theatre['position_x'].to_numpy()
ugv_position_y_theatre = ugv_data_theatre['position_y'].to_numpy()
ugv_position_z_theatre = ugv_data_theatre['position_z'].to_numpy()

# Extract UAV data from test simulation
drone_time_test = drone_data_test['time'].to_numpy()
drone_position_x_test = drone_data_test['position_x'].to_numpy()
drone_position_y_test = drone_data_test['position_y'].to_numpy()
drone_position_z_test = drone_data_test['position_z'].to_numpy()

# Extract UGV data from test simulation
ugv_time_test = ugv_data_test['time'].to_numpy()
ugv_position_x_test = ugv_data_test['position_x'].to_numpy()
ugv_position_y_test = ugv_data_test['position_y'].to_numpy()
ugv_position_z_test = ugv_data_test['position_z'].to_numpy()

# Function to set equal aspect ratio for 3D plots
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

# Function to plot 3D positions of UGV and UAV
def plot_3d(ugv_x, ugv_y, ugv_z, uav_x, uav_y, uav_z):
    """Plot 3D positions of UGV and UAV."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(ugv_x, ugv_y, ugv_z, label='UGV Position', color='b')
    ax.plot(uav_x, uav_y, uav_z, label='UAV Position', color='r')

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

# Function to plot comparative 3D positions of UGV and UAV from theatre and test data
def plot_3d_comparative(ugv_x_theatre, ugv_y_theatre, ugv_z_theatre,
                        ugv_x_test, ugv_y_test, ugv_z_test,
                        uav_x_theatre, uav_y_theatre, uav_z_theatre,
                        uav_x_test, uav_y_test, uav_z_test):
    """Plot comparative 3D positions of UGV and UAV from theatre and test data."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot UGV trajectories
    ax.plot(ugv_x_theatre, ugv_y_theatre, ugv_z_theatre, label='UGV Position (Theatre)', color='b')
    ax.plot(ugv_x_test, ugv_y_test, ugv_z_test, label='UGV Position (Simulation)', color='b', linestyle='--')

    # Plot UAV trajectories
    ax.plot(uav_x_theatre, uav_y_theatre, uav_z_theatre, label='UAV Position (Theatre)', color='r')
    ax.plot(uav_x_test, uav_y_test, uav_z_test, label='UAV Position (Simulation)', color='r', linestyle='--')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Position Comparison: UGV and UAV Theatre vs Simulation')
    ax.legend()
    ax.grid(True)

    set_axes_equal(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position_comparative_ugv_uav.png'))
    plt.show()
    plt.close()

# Function to plot 3D position of UAV from theatre and test data
def plot_uav_3d_comparative(uav_x_theatre, uav_y_theatre, uav_z_theatre,
                            uav_x_test, uav_y_test, uav_z_test):
    """Plot comparative 3D positions of UAV from theatre and test data."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot UAV trajectories
    ax.plot(uav_x_theatre, uav_y_theatre, uav_z_theatre, label='UAV Position (Theatre)', color='r')
    ax.plot(uav_x_test, uav_y_test, uav_z_test, label='UAV Position (Simulation)', color='r', linestyle='--')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Position Comparison: UAV Theatre vs Simulation')
    ax.legend()
    ax.grid(True)

    set_axes_equal(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position_comparative_uav.png'))
    plt.show()
    plt.close()

# Function to plot 3D position of UGV from theatre and test data
def plot_ugv_3d_comparative(ugv_x_theatre, ugv_y_theatre, ugv_z_theatre,
                            ugv_x_test, ugv_y_test, ugv_z_test):
    """Plot comparative 3D positions of UGV from theatre and test data."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot UGV trajectories
    ax.plot(ugv_x_theatre, ugv_y_theatre, ugv_z_theatre, label='UGV Position (Theatre)', color='b')
    ax.plot(ugv_x_test, ugv_y_test, ugv_z_test, label='UGV Position (Simulation)', color='b', linestyle='--')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Position Comparison: UGV Theatre vs Simulation')
    ax.legend()
    ax.grid(True)

    set_axes_equal(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(plot_dir, '3d_position_comparative_ugv.png'))
    plt.show()
    plt.close()

# Plot UAV position comparison over time
plt.figure(figsize=(10, 8))

# Subplot 1: X position comparison
plt.subplot(3, 1, 1)
plt.plot(drone_time_theatre, drone_position_x_theatre, label='Position X (Theatre)', color='b')
plt.plot(drone_time_test, drone_position_x_test, label='Position X (Simulation)', color='b', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('UAV Position Comparison - X')
plt.legend()
plt.grid(True)

# Subplot 2: Y position comparison
plt.subplot(3, 1, 2)
plt.plot(drone_time_theatre, drone_position_y_theatre, label='Position Y (Theatre)', color='g')
plt.plot(drone_time_test, drone_position_y_test, label='Position Y (Simulation)', color='g', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('UAV Position Comparison - Y')
plt.legend()
plt.grid(True)

# Subplot 3: Z position comparison
plt.subplot(3, 1, 3)
plt.plot(drone_time_theatre, drone_position_z_theatre, label='Position Z (Theatre)', color='r')
plt.plot(drone_time_test, drone_position_z_test, label='Position Z (Simulation)', color='r', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Z (m)')
plt.title('UAV Position Comparison - Z')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'uav_position_comparative_theatre_test.png'))
plt.close()

# Plot UGV position comparison over time
plt.figure(figsize=(10, 8))

# Subplot 1: X position comparison
plt.subplot(2, 1, 1)
plt.plot(ugv_time_theatre, ugv_position_x_theatre, label='Position X (Theatre)', color='b')
plt.plot(ugv_time_test, ugv_position_x_test, label='Position X (Simulation)', color='b', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('UGV Position Comparison - X')
plt.legend()
plt.grid(True)

# Subplot 2: Y position comparison
plt.subplot(2, 1, 2)
plt.plot(ugv_time_theatre, ugv_position_y_theatre, label='Position Y (Theatre)', color='g')
plt.plot(ugv_time_test, ugv_position_y_test, label='Position Y (Simulation)', color='g', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Position Y (m)')
plt.title('UGV Position Comparison - Y')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'ugv_position_comparative_theatre_test.png'))
plt.close()

# Plot comparative 3D positions of UGV and UAV
plot_3d_comparative(ugv_position_x_theatre, ugv_position_y_theatre, ugv_position_z_theatre,
                    ugv_position_x_test, ugv_position_y_test, ugv_position_z_test,
                    drone_position_x_theatre, drone_position_y_theatre, drone_position_z_theatre,
                    drone_position_x_test, drone_position_y_test, drone_position_z_test)

# Plot 3D positions of UGV and UAV from theatre data
plot_3d(ugv_position_x_theatre, ugv_position_y_theatre, ugv_position_z_theatre,
        drone_position_x_theatre, drone_position_y_theatre, drone_position_z_theatre)

# Plot 3D position of UAV from theatre and simulation data
plot_uav_3d_comparative(drone_position_x_theatre, drone_position_y_theatre, drone_position_z_theatre,
                        drone_position_x_test, drone_position_y_test, drone_position_z_test)

# Plot 3D position of UGV from theatre and simulation data
plot_ugv_3d_comparative(ugv_position_x_theatre, ugv_position_y_theatre, ugv_position_z_theatre,
                        ugv_position_x_test, ugv_position_y_test, ugv_position_z_test)
