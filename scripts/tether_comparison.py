#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from tqdm import tqdm
# import imageio.v2 as imageio  # No necesario si no guardamos imágenes
from pycatenary.cable import MooringLine
from pycatenary.catenary import CatenaryElastic

def calculate_catenary(x1, y1, z1, x2, y2, z2, cable_length, weight_per_meter):
    """Calcula la catenaria teórica entre dos puntos dados usando pycatenary."""
    
    w = weight_per_meter * 9.8  # N/m (peso por unidad de longitud)
    EA = 1.5e5  # (N) Módulo elástico axial

    try:
        # Vector de desplazamiento entre los dos puntos
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        # Distancias horizontal y vertical
        horizontal_distance = np.hypot(dx, dy)
        vertical_distance = dz
        straight_line_distance = np.hypot(horizontal_distance, vertical_distance)
        
        if cable_length <= straight_line_distance:
            # No se puede formar una catenaria válida
            return None, None, None

        # Crear el objeto de línea de amarre
        # Ancla en (0, 0, 0), punto de conexión en (horizontal_distance, 0, vertical_distance)
        line = MooringLine(L=cable_length, w=w, EA=EA, anchor=[0, 0, 0], fairlead=[horizontal_distance, 0, vertical_distance], floor=False)
        catenary = CatenaryElastic(line=line)
        catenary.getState(d=horizontal_distance, h=vertical_distance, floor=False)

        n_points = 100
        s_values = np.linspace(0, cable_length, n_points)
        xy = np.array([catenary.s2xy(s) for s in s_values])
        x_cat_local = xy[:, 0]  # Coordenadas locales en X
        z_cat_local = xy[:, 1]  # Coordenadas locales en Z
        y_cat_local = np.zeros_like(x_cat_local)  # Y locales son cero (plano XZ)

        # Vector unitario en la dirección horizontal
        if horizontal_distance == 0:
            ux, uy = 0, 0  # Evitar división por cero
        else:
            ux = dx / horizontal_distance
            uy = dy / horizontal_distance

        # Mapear las coordenadas locales a globales
        x_cat = x1 + x_cat_local * ux
        y_cat = y1 + x_cat_local * uy
        z_cat = z1 + z_cat_local  # Agregar z local a z1

        return x_cat, y_cat, z_cat

    except Exception as e:
        print(f"Error en calculate_catenary: {e}")
        return None, None, None

# Variable para decidir si se grafican las gráficas o no
plot_graphs = True  # Cambia a True para generar las gráficas

plot_dir = 'src/marsupial_simulator_ros2/plots'
os.makedirs(plot_dir, exist_ok=True)

tether_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/tether_data.csv')
ugv_data = pd.read_csv('src/marsupial_simulator_ros2/simulation_data/ugv_data.csv')

# Verificar y eliminar filas con valores NaN o infinitos
tether_data = tether_data.replace([np.inf, -np.inf], np.nan).dropna()
ugv_data = ugv_data.replace([np.inf, -np.inf], np.nan).dropna()

# Asegurarse de que los tiempos están ordenados
tether_data = tether_data.sort_values('time').reset_index(drop=True)
ugv_data = ugv_data.sort_values('time').reset_index(drop=True)

times = tether_data['time'].to_numpy()
link_columns = [col for col in tether_data.columns if '_x' in col]
link_names = [col.replace('_x', '') for col in link_columns]
link_positions = {link: tether_data[[f'{link}_x', f'{link}_y', f'{link}_z']].to_numpy() for link in link_names}

ugv_times = ugv_data['time'].to_numpy()
ugv_position_x = ugv_data['position_x'].to_numpy()
ugv_position_y = ugv_data['position_y'].to_numpy()
ugv_position_z = ugv_data['position_z'].to_numpy()
cable_length = ugv_data['cable_length'].to_numpy()

# Interpolación segura usando np.interp
winch_x = np.interp(times, ugv_times, ugv_position_x - 0.25, left=np.nan, right=np.nan)
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

# Filtrar link_positions para que coincida con times
for link in link_positions:
    link_positions[link] = link_positions[link][valid_indices]

x_min, x_max = tether_data[[col for col in tether_data.columns if '_x' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_x' in col]].max().max()
y_min, y_max = tether_data[[col for col in tether_data.columns if '_y' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_y' in col]].max().max()
z_min, z_max = tether_data[[col for col in tether_data.columns if '_z' in col]].min().min(), tether_data[[col for col in tether_data.columns if '_z' in col]].max().max()

threshold_distance = 0.25  
weight_per_meter = 0.1  # Ajusta este valor según las propiedades de tu cuerda

errors = []

if plot_graphs:
    # Configurar el modo interactivo
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

for i, time in tqdm(enumerate(times), total=len(times), desc="Generando frames"):
    current_winch_position = np.array([winch_x[i], winch_y[i], winch_z[i]])
    current_cable_length = cable_length_interp[i]

    valid_links = []
    for link_number in range(len(link_names), 0, -1):  
        link = f"link_{link_number}"
        if link in link_positions:
            distance_to_winch = np.linalg.norm(link_positions[link][i] - current_winch_position)
            
            if distance_to_winch > threshold_distance:
                valid_links.append(link)
            else:
                break 

    if len(valid_links) > 1:
        first_link = valid_links[0]  # Extremo del UAV
        last_link = valid_links[-1]  # Extremo del UGV

        # Calcular la longitud total considerando las longitudes diferentes de los elementos
        num_links = len(valid_links)
        if num_links <= 10:
            total_length = num_links * 0.045
        else:
            total_length = 10 * 0.045 + (num_links - 10) * 0.05

        # Calcular la catenaria teórica
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
                ax.clear()
                ax.plot(x_cat, y_cat, z_cat, 'g--', label='Catenaria Teórica')
                sc = ax.scatter(simulated_points[:, 0], simulated_points[:, 1], simulated_points[:, 2], c=distances, cmap='Reds', label='Error (m)')
        else:
            print(f"No se pudo calcular la catenaria en el tiempo {time}")
            continue  # Omitir este instante de tiempo
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

        plt.draw()
        plt.pause(0.001)  # Pausa breve para actualizar la gráfica

if plot_graphs:
    plt.ioff()
    plt.show()

errors = np.array(errors)
# Usar la mediana para evitar la influencia de valores atípicos
median_error = np.median(errors)

plt.figure(figsize=(10, 8))
plt.plot(times[:len(errors)], errors, 'r-', label='Error absoluto (m)')
plt.xlabel('Tiempo (s)')
plt.ylabel('Error absoluto (m)')
plt.title('Error Absoluto en Cada Instante')
plt.legend()
plt.grid(True)
plt.tight_layout()
error_plot_filename = os.path.join(plot_dir, 'tether_error_plot.png')
plt.savefig(error_plot_filename)
plt.close()

print(f"\033[93mMediana del error total: {median_error:.4f} m\033[0m")

print(f'Gráfico de error guardado como {error_plot_filename}')
