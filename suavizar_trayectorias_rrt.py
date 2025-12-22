import pandas as pd
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

# 1. Configuración de rutas
HERE = Path(__file__).resolve().parent
sys.path.append(str(HERE))

from python_motion_planning.utils import CurveFactory

# --- CONFIGURACIÓN DE VELOCIDAD ---
SPEED_FACTOR = 2  

def procesar_trayectoria(archivo_csv, factor_filtrado, curve_factory):
    if not os.path.exists(archivo_csv):
        print(f"Error: No se encuentra {archivo_csv}")
        return None, None, None

    df = pd.read_csv(archivo_csv)
    puntos_crudos = list(zip(df['x'], df['y']))

    # --- CORRECCIÓN: LIMPIEZA DE DUPLICADOS ---
    # RRT a veces guarda el mismo punto dos veces seguidas.
    puntos_limpios = [puntos_crudos[0]]
    
    for x, y in puntos_crudos[1:]:
        last_x, last_y = puntos_limpios[-1]
        distancia = np.hypot(x - last_x, y - last_y)
        
        # Solo lo agregamos si está a más de 1mm de distancia
        if distancia > 0.001:
            puntos_limpios.append((x, y))

    # Aplicamos el factor de filtrado
    puntos_filtrados = puntos_limpios[::factor_filtrado]
    
    # Aseguramos que el punto final siempre esté incluido
    if puntos_limpios[-1] != puntos_filtrados[-1]:
        puntos_filtrados.append(puntos_limpios[-1])

    # Generación B-Spline
    generator = curve_factory("bspline", step=0.001, k=3)
    
    try:
        # display=False para obtener la lista de coordenadas
        trayectoria_suave = generator.run(puntos_filtrados, display=False)
        return puntos_crudos, puntos_filtrados, trayectoria_suave
    except Exception as e:
        print(f"⚠️ Error matemático en {archivo_csv}: {e}")
        return None, None, None

def guardar_csv(trayectoria, nombre_archivo):
    """
    Guarda la lista de tuplas (x, y) en un archivo CSV.
    """
    if trayectoria is None:
        return
    
    df_salida = pd.DataFrame(trayectoria, columns=['x', 'y'])
    df_salida.to_csv(nombre_archivo, index=False)
    print(f"💾 Guardado exitosamente: {nombre_archivo} ({len(trayectoria)} puntos)")

def configurar_plot_completo(ax, raw, ctrl, smooth, titulo, color_curva, color_raw):
    """
    Dibuja TODO lo estático: Puntos raw, puntos control y la CURVA ENTERA.
    """
    if not smooth: return
    
    # Desempaquetar datos
    sx, sy = zip(*smooth)
    cx, cy = zip(*ctrl)
    rx, ry = zip(*raw)
    
    # 1. La Curva Suave
    ax.plot(sx, sy, '-', c=color_curva, linewidth=1.6, alpha=0.95, label="B-Spline", zorder=1)
    
    # 2. Puntos de Control
    ax.plot(cx, cy, '--', c='#aaaaaa', alpha=0.5, lw=0.8, zorder=2)
    
    # 3. Raw Waypoints
    ax.plot(rx, ry, "x", c=color_raw, markersize=5, alpha=0.6, label="Waypoints RRT", zorder=3)
    
    ax.set_title(titulo)
    ax.axis("equal")
    ax.grid(True, alpha=0.15)
    ax.legend(loc='upper right', fontsize='small')
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

def main():
    # Archivos de entrada
    archivo_05 = "rrt_path_0.5m.csv"
    archivo_10 = "rrt_path_1.0m.csv"
    
    # Archivos de salida (Nuevos CSVs suavizados)
    salida_05 = "rrt_path_0.5m_smooth.csv"
    salida_10 = "rrt_path_1.0m_smooth.csv"
    
    # Factor de filtrado
    factor_filtrado = 7
    
    print("Procesando trayectorias RRT...")
    curve_factory = CurveFactory()

    # 1. Procesar
    raw_05, ctrl_05, smooth_05 = procesar_trayectoria(archivo_05, factor_filtrado, curve_factory)
    raw_10, ctrl_10, smooth_10 = procesar_trayectoria(archivo_10, factor_filtrado, curve_factory)

    if not smooth_05 or not smooth_10:
        print("❌ No se pudieron generar las curvas debido a errores en los datos.")
        return

    # 2. GUARDAR CSVs
    print("-" * 30)
    guardar_csv(smooth_05, salida_05)
    guardar_csv(smooth_10, salida_10)
    print("-" * 30)

    # Datos para la animación
    sx_05, sy_05 = zip(*smooth_05)
    sx_10, sy_10 = zip(*smooth_10)

    # --- VISUALIZACIÓN ---
    print(f"Iniciando Animación")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Dibujar fondos
    configurar_plot_completo(ax1, raw_05, ctrl_05, smooth_05, "RRT waypoints: 0.5m (Suavizado)", "#9467bd", "red")
    configurar_plot_completo(ax2, raw_10, ctrl_10, smooth_10, "RRT waypoints: 1.0m (Suavizado)", "#17becf", "orange")

    # Objetos móviles
    car1, = ax1.plot([], [], 'bo', markersize=8, markeredgecolor='white', zorder=10, label="Carro")
    car2, = ax2.plot([], [], 'go', markersize=8, markeredgecolor='white', zorder=10, label="Carro")

    # Función de actualización
    def update(frame):
        idx = frame * SPEED_FACTOR
        
        # Carro 1
        if idx < len(sx_05):
            car1.set_data([sx_05[idx]], [sy_05[idx]])
        else:
            car1.set_data([sx_05[-1]], [sy_05[-1]])

        # Carro 2
        if idx < len(sx_10):
            car2.set_data([sx_10[idx]], [sy_10[idx]])
        else:
            car2.set_data([sx_10[-1]], [sy_10[-1]])

        return car1, car2

    max_len = max(len(sx_05), len(sx_10))
    total_frames = (max_len // SPEED_FACTOR) + 20

    ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=20, blit=True, repeat=False)

    plt.suptitle("Algoritmo RRT Suavizado + CSV Export", fontsize=16)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
