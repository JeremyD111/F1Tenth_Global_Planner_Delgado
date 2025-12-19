import os
import cv2
import csv
import yaml
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt

# Añadir el path para importar los módulos del repo
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from python_motion_planning.utils import Map, SearchFactory

# --- FUNCIONES ---

def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)

    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    if downsample_factor > 12:
        map_bin = cv2.dilate(map_bin, np.ones((5, 5), np.uint8), iterations=2)
    elif downsample_factor >= 4:
        map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)

    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    if downsample_factor > 12:
        map_bin = (map_bin > 0.10).astype(np.uint8)
    elif downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    resolution *= downsample_factor
    return map_bin, resolution, origin

def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)

def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)

# ------

def rrt_env_from_map(map_bin):
    """
    Crea el ambiente 'Map' que RRT requiere, usando la estructura
    de obstáculos que maneja internamente este repositorio.
    """
    h, w = map_bin.shape
    env = Map(w, h)
    
    # Según global_examples.py, RRT usa 'obs_rect' para colisiones.
    # Extraemos las coordenadas de los obstáculos.
    obs_y, obs_x = np.where(map_bin == 1)
    
    
    obs_rect = [[x, h - 1 - y, 1, 1] for x, y in zip(obs_x, obs_y)]
    
    
    env.update(obs_rect=obs_rect)
    return env


def get_spaced_waypoints(path, distance_m, resolution):
    if not path: return []
    path = list(reversed(path)) # RRT devuelve de goal a start, invertimos
    new_path = [path[0]]
    last_point = path[0]
    for i in range(1, len(path)):
        d = np.sqrt((path[i][0] - last_point[0])**2 + (path[i][1] - last_point[1])**2) * resolution
        if d >= distance_m:
            new_path.append(path[i])
            last_point = path[i]
    if path[-1] not in new_path:
        new_path.append(path[-1])
    return new_path

def save_path_as_csv(path, filename, resolution, origin, image_height):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in path:
            x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
            writer.writerow([round(x, 3), round(y, 3)])

# --- BLOQUE DE EJECUCIÓN ---

if __name__ == "__main__":
    base_path = Path(__file__).resolve().parent
    yaml_path = base_path / "Mapas-F1Tenth" / "BrandsHatch_map2.yaml"
    
    # Downsample factor de 10 o 12 para que RRT sea eficiente
    downsample_factor = 6

    x_start, y_start = 31.5, -39.5 
    x_goal, y_goal = 28.1, -38.8

    # 1. Cargar mapa 
    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    img_h = map_bin.shape[0]

    # 2. Crear entorno Map 
    env = rrt_env_from_map(map_bin)

    # 3. Coordenadas
    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)

    print(f"Start (map): {start}, Goal (map): {goal}")
    
    # 4. Planificador RRT
    planner = SearchFactory()("rrt", start=start, goal=goal, env=env)
    
    # Parámetros recomendados para RRT en este mapa
    planner.max_dist = 3.8 # Cuánto se estira cada rama
    planner.sample_num = 6000 
    planner.goal_sample_rate = 0.045

    print("Ejecutando Animación RRT...")
    #planner.run()

    # 5. Generar Waypoints y CSVs
    #cost, path_raw, _ = planner.plan()
    
    cost, path_raw, expand = planner.plan()
    planner.plot.animation(path_raw, "RRT", cost, expand)
    
    if path_raw:
        path_05 = get_spaced_waypoints(path_raw, 0.5, resolution)
        path_10 = get_spaced_waypoints(path_raw, 1.0, resolution)
        
        save_path_as_csv(path_05, "rrt_path_0.5m.csv", resolution, origin, img_h)
        save_path_as_csv(path_10, "rrt_path_1.0m.csv", resolution, origin, img_h)
        print("Archivos RRT generados con éxito.")
    else:
        print("No se encontró ruta con RRT.")
