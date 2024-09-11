#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial import cKDTree
from tqdm import tqdm
import imageio.v2 as imageio
from pycatenary.cable import MooringLine
from pycatenary.catenary import CatenaryElastic

# https://github.com/tridelat/pycatenary Calculo de la curva de la catenaria real

def calculate_catenary(x1, y1, z1, x2, y2, z2, cable_length, weight_per_meter):
    """Calcula la catenaria teórica entre dos puntos dados usando pycatenary."""
    
    w = weight_per_meter * cable_length # kg/m
    EA = 560e3  # (N)
    
    try:
        euclidean_distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        
        if cable_length <= euclidean_distance:
            raise ValueError("La longitud del cable es menor o igual que la distancia euclidiana entre los puntos de anclaje. No se puede formar una catenaria válida.")
        
        line = MooringLine(L=cable_length, w=w, EA=EA, anchor=[x1, y1, z1], fairlead=[x2, y2, z2], floor=False)
        
        catenary = CatenaryElastic(line=line)
        
        catenary.getState(d=euclidean_distance, h=z2 - z1, floor=False)

        n_points = 100
        s_values = np.linspace(0, cable_length, n_points)  
        xyz = np.array([catenary.s2xy(s) for s in s_values])

        z_cat = xyz[:, 1] + z1
        x_cat = np.linspace(x1, x2, n_points)
        y_cat = np.linspace(y1, y2, n_points)

        return x_cat, y_cat, z_cat

    except Exception as e:
        print(f"Error in calculate_catenary: {e}")
        return None, None, None

plot_dir = 'src/marsupial_simulator_ros2/plots'
os.makedirs(plot_dir, exist_ok=True)

tether_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/tether_data.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')

times = tether_data['time'].to_numpy()
link_columns = [col for col in tether_data.columns if '_x' in col]
link_positions = {link: tether_data[[f'{link}_x', f'{link}_y', f'{link}_z']].to_numpy() for link in [col.replace('_x', '') for col in link_columns]}

ugv_times = ugv_data['time'].to_numpy() 
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()
cable_length = ugv_data['cable_length'].to_numpy()

winch_x = interp1d(ugv_times, ugv_position_x + -0.25, bounds_error=False, fill_value="extrapolate")
winch_y = interp1d(ugv_times, ugv_position_y, bounds_error=False, fill_value="extrapolate")
winch_z = interp1d(ugv_times, ugv_position_z + 0.35, bounds_error=False, fill_value="extrapolate")
cable_length_interp = interp1d(ugv_times, cable_length, bounds_error=False, fill_value="extrapolate")

x_min, x_max = tether_data[[col for col in tether_data.columns if '_x' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_x' in col]].max().max()
y_min, y_max = tether_data[[col for col in tether_data.columns if '_y' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_y' in col]].max().max()
z_min, z_max = tether_data[[col for col in tether_data.columns if '_z' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_z' in col]].max().max()

threshold_distance = 0.25  
weight_per_meter = 0.1  

errors = []
images = []

for i, time in tqdm(enumerate(times), total=len(times), desc="Generating frames"):
    current_winch_position = np.array([winch_x(time), winch_y(time), winch_z(time)])
    current_cable_length = cable_length_interp(time)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    valid_links = []
    for link_number in range(len(link_positions), 0, -1):  
        link = f"link_{link_number}"
        if link in link_positions:
            distance_to_winch = np.linalg.norm(link_positions[link][i] - current_winch_position)
            
            if distance_to_winch > threshold_distance:
                valid_links.append(link)
            else:
                break 

    if len(valid_links) > 1:
        first_link = valid_links[0]
        last_link = valid_links[-1]

        num_links = len(valid_links)
        initial_length = min(10, num_links) * 0.13
        remaining_length = (num_links - 10) * 0.12 if num_links > 10 else 0  
        total_length = initial_length + remaining_length
        
        x_cat, y_cat, z_cat = calculate_catenary(
            link_positions[first_link][i, 0], link_positions[first_link][i, 1], link_positions[first_link][i, 2],
            link_positions[last_link][i, 0], link_positions[last_link][i, 1], link_positions[last_link][i, 2],
            total_length,
            weight_per_meter
        )
        if x_cat is not None:
            ax.plot(x_cat, y_cat, z_cat, 'g--', label='Theoretical Catenary')

            simulated_points = np.array([link_positions[link][i] for link in valid_links])
            theoretical_points = np.vstack([x_cat, y_cat, z_cat]).T
            
            tree = cKDTree(theoretical_points)
            distances, _ = tree.query(simulated_points)
            errors.append(np.mean(distances))

            ax.scatter(simulated_points[:, 0], simulated_points[:, 1], simulated_points[:, 2], c=distances, cmap='Reds', label='Error (m)')

    for j in range(len(valid_links) - 1):
        link = valid_links[j]
        next_link = valid_links[j + 1]
        ax.plot([link_positions[link][i, 0], link_positions[next_link][i, 0]],
                [link_positions[link][i, 1], link_positions[next_link][i, 1]],
                [link_positions[link][i, 2], link_positions[next_link][i, 2]], 'r-', label='Simulated Catenary' if j == 0 else "")

    ax.set_xlim([x_min, x_max])
    ax.set_ylim([y_min, y_max])
    ax.set_zlim([z_min, z_max])
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title(f'Time: {time:.2f} s')
    ax.legend()

    plt.tight_layout()

    frame_filename = os.path.join(plot_dir, f'frame_{i:04d}.png')
    plt.savefig(frame_filename)
    plt.close(fig)
    
    images.append(imageio.imread(frame_filename))

gif_filename = os.path.join(plot_dir, 'tether_animation_comparison.gif')
imageio.mimsave(gif_filename, images, fps=2)

errors = np.array(errors)
mean_error = np.mean(errors)

plt.figure(figsize=(10, 8))
plt.plot(times[:len(errors)], errors, 'r-', label='Error absoluto (m)')
plt.xlabel('Time (s)')
plt.ylabel('Error absoluto (m)')
plt.title('Error Absoluto en Cada Instante')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'tether_error_plot.png'))
plt.close()

print(f"\033[93mMedia del error total: {mean_error:.4f} m\033[0m")

for frame_filename in [os.path.join(plot_dir, f'frame_{i:04d}.png') for i in range(len(times))]:
    os.remove(frame_filename)

print(f'GIF saved as {gif_filename}')
print(f'Error plot saved as {os.path.join(plot_dir, "error_plot.png")}')
