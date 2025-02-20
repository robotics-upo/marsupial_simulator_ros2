#!/usr/bin/env python3

import os
import pandas as pd
import math
import yaml
import numpy as np

# Load the CSV files for UGV and UAV
drone_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/uav_data_theatre.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data_theatre.csv')

# Extract time data for UGV and UAV
ugv_time = ugv_data['time'].to_numpy()
uav_time = drone_data['time'].to_numpy()

# Extract necessary data from UAV
drone_position_x = drone_data['position_x'].to_numpy()
drone_position_y = drone_data['position_y'].to_numpy()
drone_position_z = drone_data['position_z'].to_numpy()

# Filter UAV data to remove points with Z < 1
valid_uav_indices = np.where(drone_position_z >= 1)[0]
drone_position_x = drone_position_x[valid_uav_indices]
drone_position_y = drone_position_y[valid_uav_indices]
drone_position_z = drone_position_z[valid_uav_indices]
uav_time = uav_time[valid_uav_indices]  # Filter corresponding times

# Extract necessary data from UGV
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()

# Total number of points in the CSV
total_points_ugv = len(ugv_position_x)
total_points_uav = len(drone_position_x)

# Desired number of original points
num_points_to_store = 100

# Create a uniform sampling of indices for UGV
indices_ugv = np.linspace(0, total_points_ugv - 1, num_points_to_store, dtype=int)

# Create dictionary for the YAML structure
data = {
    'marsupial_ugv': {},
    'marsupial_uav': {},
    'tether': {}
}

count = 0  # Initialize the counter

# Loop over each pair of consecutive indices
for i in range(len(indices_ugv) - 1):
    idx_start = indices_ugv[i]
    idx_end = indices_ugv[i+1]

    # Get UGV positions
    pos_start_ugv = np.array([ugv_position_x[idx_start], ugv_position_y[idx_start], ugv_position_z[idx_start]])
    pos_end_ugv = np.array([ugv_position_x[idx_end], ugv_position_y[idx_end], ugv_position_z[idx_end]])

    # Get times
    ugv_time_start = ugv_time[idx_start]
    ugv_time_end = ugv_time[idx_end]

    # Find closest UAV indices
    closest_uav_index_start = np.abs(uav_time - ugv_time_start).argmin()
    closest_uav_index_end = np.abs(uav_time - ugv_time_end).argmin()

    # Get UAV positions
    pos_start_uav = np.array([drone_position_x[closest_uav_index_start], drone_position_y[closest_uav_index_start], drone_position_z[closest_uav_index_start]])
    pos_end_uav = np.array([drone_position_x[closest_uav_index_end], drone_position_y[closest_uav_index_end], drone_position_z[closest_uav_index_end]])

    # Interpolate positions at t = 0, 1/3, 2/3
    for t in [0.0, 1.0/3.0, 2.0/3.0]:
        # UGV interpolated position
        pos_ugv = pos_start_ugv + (pos_end_ugv - pos_start_ugv) * t
        # UAV interpolated position
        pos_uav = pos_start_uav + (pos_end_uav - pos_start_uav) * t

        # Save UGV data
        data['marsupial_ugv'][f'poses{count}'] = {
            'header': f'ugv{count}',
            'seq': count,
            'frame_id': 'ugv',
            'pose': {
                'position': {
                    'x': float(pos_ugv[0]),
                    'y': float(pos_ugv[1]),
                    'z': float(pos_ugv[2])
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 0.0
                }
            }
        }

        # Save UAV data
        data['marsupial_uav'][f'poses{count}'] = {
            'header': f'uav{count}',
            'seq': count,
            'frame_id': 'uav',
            'pose': {
                'position': {
                    'x': float(pos_uav[0]),
                    'y': float(pos_uav[1]),
                    'z': float(pos_uav[2])
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 0.0
                }
            }
        }

        # Calculate tether length
        tether_length = math.sqrt(
            (pos_ugv[0] - pos_uav[0])**2 +
            (pos_ugv[1] - pos_uav[1])**2 +
            (pos_ugv[2] - pos_uav[2])**2
        )

        # Save tether data
        data['tether'][f'length{count}'] = {
            'header': f'tether{count}',
            'seq': count,
            'frame_id': 'tether_length',
            'length': tether_length
        }

        count += 1

# After the loop, add the final point at t = 1
idx_last = indices_ugv[-1]
# Get UGV final position
pos_ugv = np.array([ugv_position_x[idx_last], ugv_position_y[idx_last], ugv_position_z[idx_last]])

# Get time
ugv_time_last = ugv_time[idx_last]

# Find closest UAV index
closest_uav_index = np.abs(uav_time - ugv_time_last).argmin()

# Get UAV final position
pos_uav = np.array([drone_position_x[closest_uav_index], drone_position_y[closest_uav_index], drone_position_z[closest_uav_index]])

# Save final UGV data
data['marsupial_ugv'][f'poses{count}'] = {
    'header': f'ugv{count}',
    'seq': count,
    'frame_id': 'ugv',
    'pose': {
        'position': {
            'x': float(pos_ugv[0]),
            'y': float(pos_ugv[1]),
            'z': float(pos_ugv[2])
        },
        'orientation': {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'w': 0.0
        }
    }
}

# Save final UAV data
data['marsupial_uav'][f'poses{count}'] = {
    'header': f'uav{count}',
    'seq': count,
    'frame_id': 'uav',
    'pose': {
        'position': {
            'x': float(pos_uav[0]),
            'y': float(pos_uav[1]),
            'z': float(pos_uav[2])
        },
        'orientation': {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'w': 0.0
        }
    }
}

# Calculate tether length for the final point
tether_length = math.sqrt(
    (pos_ugv[0] - pos_uav[0])**2 +
    (pos_ugv[1] - pos_uav[1])**2 +
    (pos_ugv[2] - pos_uav[2])**2
)

# Save final tether data
data['tether'][f'length{count}'] = {
    'header': f'tether{count}',
    'seq': count,
    'frame_id': 'tether_length',
    'length': tether_length
}

count += 1  # Increment count for the final point

# Add the total number of points saved
data['marsupial_ugv']['size'] = count
data['marsupial_uav']['size'] = count
data['tether']['size'] = count

# Save to YAML file
yaml_file_path = '/home/upo/marsupial/src/marsupial_simulator_ros2/optimized_path/teatro_trajectory.yaml'
os.makedirs(os.path.dirname(yaml_file_path), exist_ok=True)

with open(yaml_file_path, 'w') as yaml_file:
    yaml.dump(data, yaml_file, default_flow_style=False)

print(f"YAML file saved in: {yaml_file_path}")
