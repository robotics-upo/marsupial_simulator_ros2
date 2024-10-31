#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from tqdm import tqdm
import imageio.v2 as imageio
from pycatenary.cable import MooringLine
from pycatenary.catenary import CatenaryElastic

def calculate_catenary(x1, y1, z1, x2, y2, z2, cable_length, weight_per_meter):
    """Calcula la catenaria teórica entre dos puntos dados usando pycatenary."""
    
    w = weight_per_meter * 9.8 # N/m (peso por unidad de longitud)
    EA = 1.5e5  # (N) Módulo elástico axial
    
    try:
        # Calcular la distancia horizontal y vertical correctamente
        horizontal_distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        vertical_distance = z2 - z1
        straight_line_distance = np.sqrt(horizontal_distance**2 + vertical_distance**2)
        
        if cable_length <= straight_line_distance:
            # No se puede formar una catenaria válida
            # print(f"Cable demasiado corto para formar una catenaria en la longitud dada.")
            return None, None, None

        # Crear el objeto de línea de amarre
        line = MooringLine(L=cable_length, w=w, EA=EA, anchor=[0, 0, z1], fairlead=[horizontal_distance, 0, z2], floor=False)
        catenary = CatenaryElastic(line=line)
        catenary.getState(d=horizontal_distance, h=vertical_distance, floor=False)

        n_points = 100
        s_values = np.linspace(0, cable_length, n_points)
        xy = np.array([catenary.s2xy(s) for s in s_values])
        x_cat_local = xy[:, 0]
        y_cat_local = xy[:, 1]
        z_cat = np.linspace(z1, z2, n_points)

        # Rotar y trasladar las coordenadas locales al sistema global
        angle = np.arctan2(y2 - y1, x2 - x1)
        x_cat = x1 + x_cat_local * np.cos(angle) - y_cat_local * np.sin(angle)
        y_cat = y1 + x_cat_local * np.sin(angle) + y_cat_local * np.cos(angle)
        
        return x_cat, y_cat, z_cat

    except Exception as e:
        print(f"Error en calculate_catenary: {e}")
        return None, None, None

# Variable para decidir si se grafican las gráficas o no
plot_graphs = False  # Cambia a True para generar las gráficas

plot_dir = 'src/marsupial_simulator_ros2/plots'
os.makedirs(plot_dir, exist_ok=True)

tether_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/tether_data.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')

# Verificar y eliminar filas con valores NaN o infinitos
tether_data = tether_data.replace([np.inf, -np.inf], np.nan).dropna()
ugv_data = ugv_data.replace([np.inf, -np.inf], np.nan).dropna()

times = tether_data['time'].to_numpy()
link_columns = [col for col in tether_data.columns if '_x' in col]
link_positions = {link: tether_data[[f'{link}_x', f'{link}_y', f'{link}_z']].to_numpy() for link in [col.replace('_x', '') for col in link_columns]}

ugv_times = ugv_data['time'].to_numpy()
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()
cable_length = ugv_data['cable_length'].to_numpy()

# Interpolación segura usando np.interp
winch_x = np.interp(times, ugv_times, ugv_position_x + -0.25, left=np.nan, right=np.nan)
winch_y = np.interp(times, ugv_times, ugv_position_y, left=np.nan, right=np.nan)
winch_z = np.interp(times, ugv_times, ugv_position_z + 0.35, left=np.nan, right=np.nan)
cable_length_interp = np.interp(times, ugv_times, cable_length, left=np.nan, right=np.nan)

# Filtrar índices válidos
valid_indices = ~np.isnan(winch_x) & ~np.isnan(winch_y) & ~np.isnan(winch_z) & ~np.isnan(cable_length_interp)
times = times[valid_indices]
winch_x = winch_x[valid_indices]
winch_y = winch_y[valid_indices]
winch_z = winch_z[valid_indices]
cable_length_interp = cable_length_interp[valid_indices]

x_min, x_max = tether_data[[col for col in tether_data.columns if '_x' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_x' in col]].max().max()
y_min, y_max = tether_data[[col for col in tether_data.columns if '_y' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_y' in col]].max().max()
z_min, z_max = tether_data[[col for col in tether_data.columns if '_z' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_z' in col]].max().max()

threshold_distance = 0.25  
weight_per_meter = 0.1  

errors = []
images = []

for i, time in tqdm(enumerate(times), total=len(times), desc="Generando frames"):
    current_winch_position = np.array([winch_x[i], winch_y[i], winch_z[i]])
    current_cable_length = cable_length_interp[i]

    if plot_graphs:
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
            simulated_points = np.array([link_positions[link][i] for link in valid_links])
            theoretical_points = np.vstack([x_cat, y_cat, z_cat]).T
            
            tree = cKDTree(theoretical_points)
            distances, _ = tree.query(simulated_points)
            if np.isfinite(np.mean(distances)):
                errors.append(np.mean(distances))
            else:
                print(f"Invalid error at time {time}: {np.mean(distances)}")

            if plot_graphs:
                ax.plot(x_cat, y_cat, z_cat, 'g--', label='Catenaria Teórica')
                sc = ax.scatter(simulated_points[:, 0], simulated_points[:, 1], simulated_points[:, 2], c=distances, cmap='Reds', label='Error (m)')
    else:
        print(f"No hay suficientes enlaces válidos en el tiempo {time}")
        continue  # Omitir este instante de tiempo

    if plot_graphs:
        for j in range(len(valid_links) - 1):
            link = valid_links[j]
            next_link = valid_links[j + 1]
            ax.plot([link_positions[link][i, 0], link_positions[next_link][i, 0]],
                    [link_positions[link][i, 1], link_positions[next_link][i, 1]],
                    [link_positions[link][i, 2], link_positions[next_link][i, 2]], 'r-', label='Catenaria Simulada' if j == 0 else "")

        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])
        ax.set_xlabel('Posición X (m)')
        ax.set_ylabel('Posición Y (m)')
        ax.set_zlabel('Posición Z (m)')
        ax.set_title(f'Tiempo: {time:.2f} s')
        if i == 0:  # Mostrar la leyenda solo una vez
            ax.legend()

        plt.tight_layout()

        frame_filename = os.path.join(plot_dir, f'frame_{i:04d}.png')
        plt.savefig(frame_filename)
        plt.close(fig)
        images.append(imageio.imread(frame_filename))
    else:
        pass  # No generar gráficas

if plot_graphs:
    gif_filename = os.path.join(plot_dir, 'tether_animation_comparison.gif')
    imageio.mimsave(gif_filename, images, fps=2)

errors = np.array(errors)
# Usar la mediana para evitar la influencia de valores atípicos
mean_error = np.median(errors)

plt.figure(figsize=(10, 8))
plt.plot(times[:len(errors)], errors, 'r-', label='Error absoluto (m)')
plt.xlabel('Tiempo (s)')
plt.ylabel('Error absoluto (m)')
plt.title('Error Absoluto en Cada Instante')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, 'tether_error_plot.png'))
plt.close()

print(f"\033[93mMedia del error total: {mean_error:.4f} m\033[0m")

if plot_graphs:
    for frame_filename in [os.path.join(plot_dir, f'frame_{i:04d}.png') for i in range(len(times))]:
        if os.path.exists(frame_filename):
            os.remove(frame_filename)

    print(f'GIF guardado como {gif_filename}')
    print(f'Gráfico de error guardado como {os.path.join(plot_dir, "tether_error_plot.png")}')
