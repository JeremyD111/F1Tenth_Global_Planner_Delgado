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
SPEED_FACTOR = 1 # Aumenta para ir más rápido

def procesar_trayectoria(archivo_csv, factor_filtrado, curve_factory):
    if not os.path.exists(archivo_csv):
        print(f"❌ Error: No se encuentra {archivo_csv}")
        return None, None, None

    df = pd.read_csv(archivo_csv)
    puntos_todos = list(zip(df['x'], df['y']))

    # Filtrado
    puntos_filtrados = puntos_todos[::factor_filtrado]
    if puntos_todos[-1] != puntos_filtrados[-1]:
        puntos_filtrados.append(puntos_todos[-1])

    # Generación
    generator = curve_factory("bspline", step=0.001, k=3)
    # display=False devuelve la lista de puntos [(x,y), (x,y)...]
    trayectoria_suave = generator.run(puntos_filtrados, display=False)
    
    return puntos_todos, puntos_filtrados, trayectoria_suave

def guardar_csv(trayectoria, nombre_archivo):
    """
    Guarda la lista de tuplas (x, y) en un archivo CSV.
    """
    if trayectoria is None:
        return
    
    # Convertimos la lista de tuplas a DataFrame de Pandas
    df_salida = pd.DataFrame(trayectoria, columns=['x', 'y'])
    
    # Guardamos sin el índice (0, 1, 2...)
    df_salida.to_csv(nombre_archivo, index=False)
    print(f"💾 Guardado exitosamente: {nombre_archivo} ({len(trayectoria)} puntos)")

def configurar_plot_completo(ax, raw, ctrl, smooth, titulo, color_curva, color_raw):
    """
    Dibuja TODO lo estático: Puntos raw, puntos control y la CURVA ENTERA.
    """
    # Desempaquetar datos
    sx, sy = zip(*smooth)
    cx, cy = zip(*ctrl)
    rx, ry = zip(*raw)
    
    # 1. La Curva Suave (Ahora es estática y más fina)
    ax.plot(sx, sy, '-', c=color_curva, linewidth=1.6, alpha=0.95, label="B-Spline", zorder=1)
    
    # 2. Puntos de Control (Gris tenue)
    ax.plot(cx, cy, '--', c='#aaaaaa', alpha=0.5, lw=0.8, zorder=2)
    
    # 3. Raw Waypoints (Dijkstra)
    ax.plot(rx, ry, "x", c=color_raw, markersize=4, alpha=0.35, label="Dijkstra", zorder=3)
    
    ax.set_title(titulo)
    ax.axis("equal")
    ax.grid(True, alpha=0.15)
    ax.legend(loc='upper right', fontsize='small')
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

def main():
    # Nombres de entrada
    archivo_05 = "f1tenth/dijkstra_0.5m.csv"
    archivo_10 = "f1tenth/dijkstra_1.0m.csv"
    
    # Nombres de salida (Suavizados)
    salida_05 = "f1tenth/dijkstra_0.5m_smooth.csv"
    salida_10 = "f1tenth/dijkstra_1.0m_smooth.csv"
    
    factor_filtrado = 7
    
    print("⚙️  Procesando trayectorias...")
    curve_factory = CurveFactory()

    # 1. Procesar
    raw_05, ctrl_05, smooth_05 = procesar_trayectoria(archivo_05, factor_filtrado, curve_factory)
    raw_10, ctrl_10, smooth_10 = procesar_trayectoria(archivo_10, factor_filtrado, curve_factory)

    if not smooth_05 or not smooth_10:
        print("❌ Error en el procesamiento de alguna trayectoria.")
        return

    # 2. GUARDAR EN CSV (NUEVO PASO)
    print("-" * 30)
    guardar_csv(smooth_05, salida_05)
    guardar_csv(smooth_10, salida_10)
    print("-" * 30)

    # Datos para la animación
    sx_05, sy_05 = zip(*smooth_05)
    sx_10, sy_10 = zip(*smooth_10)

    # --- VISUALIZACIÓN ---
    print(f"🚀 Iniciando Animación...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Dibujar fondos
    configurar_plot_completo(ax1, raw_05, ctrl_05, smooth_05, "Dijkstra 0.5m (Suavizado)", "#1f77b4", "red")
    configurar_plot_completo(ax2, raw_10, ctrl_10, smooth_10, "Dijkstra 1.0m (Suavizado)", "#2ca02c", "orange")

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
    total_frames = (max_len // SPEED_FACTOR) + 5

    ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=20, blit=True, repeat=False)

    plt.suptitle("Comparativa y Generación de CSVs Suavizados", fontsize=16)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
