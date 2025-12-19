import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, SearchFactory
from pathlib import Path


def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)  # asegurar Path
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)


    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar: 1 = ocupado, 0 = libre
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Engrosar obstáculos según el factor
    if downsample_factor > 12:
        map_bin = cv2.dilate(map_bin, np.ones((5, 5), np.uint8), iterations=2)
    elif downsample_factor >= 4:
        map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)
    # para 1-3 no se dilata

    # Downsampling con interpolación adecuada
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Re-binarizar según nivel
    if downsample_factor > 12:
        map_bin = (map_bin > 0.10).astype(np.uint8)
    elif downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    # Ajustar resolución
    resolution *= downsample_factor

    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)
    obstacles = {(x, h - 1 - y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
    env.update(obstacles)
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)

def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)


def filter_path_by_distance(path, target_distance, resolution):
    """Filtra el path para que los puntos tengan una separación mínima en metros."""
    if not path: return []
    # Invertimos el path de una vez para trabajar de Start a Goal
    path = list(reversed(path))
    new_path = [path[0]]
    last_point = path[0]
    
    for i in range(1, len(path)):
        # Distancia euclidiana en píxeles convertida a metros
        d = np.sqrt((path[i][0] - last_point[0])**2 + (path[i][1] - last_point[1])**2) * resolution
        if d >= target_distance:
            new_path.append(path[i])
            last_point = path[i]
            
    # Asegurar que el punto final esté incluido
    if path[-1] not in new_path:
        new_path.append(path[-1])
    return new_path

def save_custom_csv(path, filename, resolution, origin, image_height):

    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in path:
            x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
            writer.writerow([round(x,2),round(y,2)])


#--- BLOQUE PRINCIPAL ---

if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "BrandsHatch_map2.yaml"
    downsample_factor = 7 

    x_start, y_start = 31, -39.9  
    x_goal, y_goal = 28.1, -38.8

    # Carga inicial
    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    env = grid_from_map(map_bin)
    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)
    img_h = map_bin.shape[0]

    # --- EJECUCIÓN 1: WAYPOINTS 0.5m ---
    print(f"\n--- Ejecutando Dijkstra para Waypoints 0.5m ---")
    planner_05 = SearchFactory()("dijkstra", start=start, goal=goal, env=env)
    planner_05.run() # Primera animación
    
    _, path_raw, _ = planner_05.plan()
    path_05 = filter_path_by_distance(path_raw, 0.5, resolution)
    save_custom_csv(path_05, "dijkstra_0.5m.csv", resolution, origin, img_h)
    print(f"Ruta 0.5m guardada. Puntos generados: {len(path_05)}")

    # --- EJECUCIÓN 2: WAYPOINTS 1.0m ---
    print(f"\n--- Ejecutando Dijkstra para Waypoints 1.0m ---")
    planner_10 = SearchFactory()("dijkstra", start=start, goal=goal, env=env)
    planner_10.run() # Segunda animación
    
    _, path_raw, _ = planner_10.plan()
    path_10 = filter_path_by_distance(path_raw, 1.0, resolution)
    save_custom_csv(path_10, "dijkstra_1.0m.csv", resolution, origin, img_h)
    print(f"Ruta 1.0m guardada. Puntos generados: {len(path_10)}")
    
    
    
