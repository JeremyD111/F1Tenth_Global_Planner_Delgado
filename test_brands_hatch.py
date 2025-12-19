import os
import sys
import cv2
import matplotlib.pyplot as plt

# Añadimos la carpeta del proyecto al path para poder importar los módulos
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from python_motion_planning.utils import Map

def main():
    # 1. Cargar el mapa de BrandsHatch
    map_path = "Mapas-F1Tenth/BrandsHatch_map.png"
    
    # Leemos la imagen en escala de grises
    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    
    # En ROS, el negro (0) es obstáculo y el blanco (255) es libre.
    # A veces hay que binarizar para asegurar limpieza.
    _, map_binarized = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)

    plt.imshow(map_binarized, cmap='gray')
    plt.title("Mapa de BrandsHatch - Selecciona coordenadas")
    plt.show()

if __name__ == '__main__':
    main()
