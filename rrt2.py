import os
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import csv # Importar para guardar CSV

# --- FUNCIONES DE CARGA Y TRANSFORMACIÓN DE MAPA ---

def load_map_optimized(yaml_path, factor=5):
    """
    Carga el mapa de un archivo YAML, lo binariza, lo dilata para engrosar los obstáculos
    y lo redimensiona para una mayor eficiencia en el RRT.
    """
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    img_path = Path(yaml_path).parent / config['image']
    map_img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    
    # 1. Binarizar: 1 es obstáculo (negro), 0 es libre (blanco)
    # Umbral más estricto para captar mejor los bordes y limpiar ruidos.
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < 150] = 1 
    
    # 2. Engrosar las líneas de la pista (DILATACIÓN)
    # Esto asegura que las líneas negras sean continuas y no se rompan al reducir el tamaño.
    kernel = np.ones((5,5), np.uint8) 
    map_bin = cv2.dilate(map_bin, kernel, iterations=1)
    
    # 3. Limpieza morfológica: Elimina píxeles negros sueltos dentro de la pista blanca (ruido).
    map_bin = cv2.morphologyEx(map_bin, cv2.MORPH_OPEN, kernel)
    
    # 4. Redimensionar (downsample) para acelerar los cálculos de colisión del RRT
    h, w = map_bin.shape
    map_small = cv2.resize(map_bin, (w // factor, h // factor), interpolation=cv2.INTER_NEAREST)
    
    # 5. Asegurarse de que después del resize los puntos negros sigan siendo 1.
    map_small = (map_small > 0).astype(np.uint8)
    
    resolution = config['resolution'] * factor
    origin = config['origin']
    return map_small, resolution, origin

def world_to_map(x_world, y_world, resolution, origin):
    """Convierte coordenadas del mundo real a coordenadas del mapa binarizado."""
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return np.array([x_map, y_map]) # Retornar como array numpy para facilidad

def map_to_world(x_map, y_map, resolution, origin):
    """Convierte coordenadas del mapa a coordenadas del mundo real."""
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return x_world, y_world

# --- CLASE RRT INDEPENDIENTE Y OPTIMIZADA ---
class FastRRT:
    def __init__(self, map_bin, start, goal, step_len=8, goal_sample_rate=0.4, max_iter=8000):
        self.map = map_bin
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.step_len = step_len
        self.goal_rate = goal_sample_rate
        self.max_iter = max_iter
        self.nodes = [self.start] # Lista de nodos [x, y]
        self.parent = {tuple(self.start): None} # Diccionario para reconstruir el camino: {hijo: padre}

    def is_collision(self, p1, p2):
        """
        Verifica colisiones a lo largo de una línea entre dos puntos usando la matriz del mapa.
        Mucho más rápido que iterar sobre una lista de rectángulos.
        """
        steps = int(np.linalg.norm(p2 - p1) / self.map.shape[0] * 50) + 2 # Más puntos para chequear
        if steps == 0: steps = 2 # Evitar divisiones por cero si p1 y p2 son iguales
        
        for i in range(steps):
            p = p1 + (p2 - p1) * (i / (steps -1)) # Interpolación lineal
            x, y = int(p[0]), int(self.map.shape[0] - 1 - p[1]) # Inversión del eje Y para grid
            
            # Chequea límites del mapa
            if not (0 <= x < self.map.shape[1] and 0 <= y < self.map.shape[0]):
                return True # Fuera de límites es colisión
            
            # Chequea si el píxel es un obstáculo
            if self.map[y, x] == 1:
                return True
        return False

    def plan(self):
        """Implementación del algoritmo RRT para encontrar una ruta."""
        for i in range(self.max_iter):
            # 1. Muestreo aleatorio (con sesgo a meta para guiar el camino)
            if np.random.random() < self.goal_rate:
                rnd = self.goal
            else:
                rnd = np.array([np.random.uniform(0, self.map.shape[1]), 
                               np.random.uniform(0, self.map.shape[0])])

            # 2. Encontrar el nodo más cercano en el árbol
            dists = [np.linalg.norm(n - rnd) for n in self.nodes]
            nearest_node = self.nodes[np.argmin(dists)]

            # 3. Extender el árbol hacia el punto aleatorio
            theta = np.arctan2(rnd[1] - nearest_node[1], rnd[0] - nearest_node[0])
            new_node = nearest_node + np.array([self.step_len * np.cos(theta), 
                                               self.step_len * np.sin(theta)])

            # 4. Verificación de colisión y añadir nodo si es válido
            if not self.is_collision(nearest_node, new_node):
                self.nodes.append(new_node)
                self.parent[tuple(new_node)] = nearest_node

                # 5. Comprobar si se ha llegado a la meta
                if np.linalg.norm(new_node - self.goal) <= self.step_len:
                    if not self.is_collision(new_node, self.goal): # Última colisión
                        self.parent[tuple(self.goal)] = new_node
                        return self.extract_path()
        return None # No se encontró camino

    def extract_path(self):
        """Reconstruye el camino desde la meta hasta el inicio."""
        path = []
        curr = tuple(self.goal)
        while curr is not None:
            path.append(curr)
            parent = self.parent.get(curr) # Usar .get() para evitar KeyError si el nodo final no tiene padre
            curr = tuple(parent) if parent is not None else None
        return path[::-1] # Invertir para ir de inicio a meta

# --- FUNCIONES DE WAYPOINTS Y CSV ---

def get_spaced_waypoints(path, distance_m, resolution):
    """
    Espacia los waypoints a una distancia mínima especificada.
    El path viene en coordenadas de mapa.
    """
    if not path or len(path) < 2: return path
    
    # path ya viene de start a goal
    new_path = [path[0]]
    last_point = path[0]
    
    for i in range(1, len(path)):
        # Calcula la distancia euclidiana entre el punto actual y el último punto añadido
        d = np.linalg.norm(np.array(path[i]) - np.array(last_point)) * resolution
        
        if d >= distance_m:
            new_path.append(path[i])
            last_point = path[i]
            
    # Asegurarse de que el punto final siempre esté incluido
    if tuple(path[-1]) not in [tuple(p) for p in new_path]:
        new_path.append(path[-1])
        
    return new_path

def save_path_as_csv(path_map_coords, filename, resolution, origin, img_height):
    """Guarda el camino en un archivo CSV en coordenadas del mundo real."""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in path_map_coords:
            x_world, y_world = map_to_world(x_map, y_map, resolution, origin)
            writer.writerow([round(x_world, 3), round(y_world, 3)]) # Redondear a 3 decimales

# --- BLOQUE DE EJECUCIÓN PRINCIPAL ---

if __name__ == "__main__":
    base_path = Path(__file__).resolve().parent
    yaml_file = base_path / "Mapas-F1Tenth" / "BrandsHatch_map2.yaml"
    
    # Factor de downsample (5 es un buen equilibrio entre detalle y velocidad)
    factor = 5 
    map_bin_small, resolution, origin = load_map_optimized(str(yaml_file), factor)

    # --- COORDENADAS DE INICIO Y META (AJUSTADAS PARA ESTAR DENTRO DE LA PISTA) ---
    # Es crucial que estas coordenadas estén en un área "blanca" del mapa
    # Ajusta estos valores si el RRT no encuentra la ruta, moviéndolos ligeramente
    start_world_x, start_world_y = 30.5, -39.0 # Ligeramente ajustado
    goal_world_x, goal_world_y = 25.0, -38.5   # Ligeramente ajustado

    start_map_coords = world_to_map(start_world_x, start_world_y, resolution, origin)
    goal_map_coords = world_to_map(goal_world_x, goal_world_y, resolution, origin)

    print(f"Iniciando RRT en BrandsHatch...")
    print(f"Inicio (mundo): ({start_world_x:.2f}, {start_world_y:.2f}) -> (mapa): {start_map_coords}")
    print(f"Meta (mundo): ({goal_world_x:.2f}, {goal_world_y:.2f}) -> (mapa): {goal_map_coords}")

    # Inicializar el RRT con los parámetros optimizados
    rrt = FastRRT(map_bin_small, start_map_coords, goal_map_coords, 
                  step_len=10,        # Largo de cada paso del RRT
                  goal_sample_rate=0.3, # Probabilidad de muestrear la meta
                  max_iter=10000)     # Número máximo de intentos

    path_found_map_coords = rrt.plan() # Ejecutar el algoritmo

    # --- VISUALIZACIÓN DEL RESULTADO ---
    plt.figure(figsize=(12, 10))
    
    # Dibujar el mapa binarizado (con la pista sólida)
    plt.imshow(map_bin_small, cmap='gray_r', origin='lower')
    
    # Dibujar el árbol de expansión de RRT (ramas cian sutiles)
    for node_coords, parent_coords in rrt.parent.items():
        if parent_coords is not None:
            plt.plot([node_coords[0], parent_coords[0]], 
                     [node_coords[1], parent_coords[1]], 'cyan', alpha=0.15, lw=0.5)

    if path_found_map_coords is not None:
        print("¡Camino encontrado con RRT! Trazando la ruta...")
        path_np = np.array(path_found_map_coords)
        plt.plot(path_np[:, 0], path_np[:, 1], 'red', lw=3, label="Trayectoria RRT")
        
        # Generar los archivos CSV con waypoints espaciados
        path_05m = get_spaced_waypoints(path_found_map_coords, 0.5, resolution)
        path_10m = get_spaced_waypoints(path_found_map_coords, 1.0, resolution)
        
        save_path_as_csv(path_05m, "rrt_path_0.5m.csv", resolution, origin)
        save_path_as_csv(path_10m, "rrt_path_1.0m.csv", resolution, origin)
        print("Archivos 'rrt_path_0.5m.csv' y 'rrt_path_1.0m.csv' generados.")

    else:
        print("El RRT no encontró un camino. Revisa las coordenadas de inicio/meta o el mapa.")

    # Dibujar los puntos de inicio y meta
    plt.scatter(start_map_coords[0], start_map_coords[1], c='green', s=150, 
                label="Inicio", edgecolors='black', zorder=5)
    plt.scatter(goal_map_coords[0], goal_map_coords[1], c='blue', s=150, 
                label="Meta", edgecolors='black', zorder=5)
    
    # Ajuste de los límites del plot para hacer zoom a la pista
    y_idx, x_idx = np.where(map_bin_small == 1)
    if len(x_idx) > 0:
        plt.xlim(min(x_idx) - 20, max(x_idx) + 20)
        plt.ylim(min(y_idx) - 20, max(y_idx) + 20)

    plt.legend()
    plt.title("RRT BrandsHatch - Jeremy Delgado (Ruta Final)")
    plt.xlabel("Coordenada X (píxeles del mapa)")
    plt.ylabel("Coordenada Y (píxeles del mapa)")
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    plt.show()
