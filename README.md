# Parte A: Global Path Planning – RRT y Dijkstra aplicado a F1TENTH

El trabajo se basa en el repositorio original:

**Global_Planner:** https://github.com/widegonz/Global_Planner

A partir de este repositorio base, se realizaron adaptaciones y modificaciones para:

- Implementar el algoritmo **Dijkstra** en el mapa BrandsHatch.
- Implementar el algoritmo **RRT (Rapidly-exploring Random Tree)** en el mapa BrandsHatch.
- Generar trayectorias y waypoints exportables para navegación.
- Suavizar las trayectorias generadas por Dijkstra y RRT.


## Descripción general del algoritmo asignado: Dijkstra

El algoritmo de **Dijkstra** permite encontrar el camino de costo mínimo entre un nodo inicial y todos los demás nodos de un grafo ponderado, siempre que los costos de las aristas sean no negativos.

Dijkstra se utiliza para buscar la **ruta óptima** entre un punto inicial y un punto objetivo sobre una representación discreta del entorno, generalmente modelada como una grilla ocupacional. Cada celda del mapa corresponde a un nodo del grafo, y las transiciones entre celdas adyacentes representan las aristas del grafo.










## Funcionamiento del algoritmo Dijkstra en Python
### 1. Script principal: `f1tenth_map.py`
Este script se encarga de cargar el mapa, crear el entorno, ejecutar el algoritmo Dijkstra y exportar la trayectoria generada.

#### Función `load_map(yaml_path, downsample_factor)`
Carga un mapa a partir de un archivo YAML, binariza la imagen del mapa para distinguir obstáculos y espacio libre, aplica un proceso de engrosamiento de obstáculos y realiza un downsampling de la imagen para reducir la complejidad computacional.

**Entradas:**
- `yaml_path`: ruta al archivo `.yaml` del mapa.
- `downsample_factor: factor de reducción de resolución del mapa.

**Salidas:**
- `map_bin`: mapa binarizado (0: libre, 1: ocupado).
- `resolution`: resolución ajustada del mapa en metros por celda.
- `origin`: origen del mapa en coordenadas del mundo.


#### Función `grid_from_map(map_bin)`
Convierte el mapa binarizado en una estructura de tipo Grid, utilizada por el planificador. Las celdas ocupadas se registran como obstáculos.

**Entradas:**
- `map_bin`: matriz binaria del mapa.

**Salidas:**
- `env`: objeto `Grid` que representa el entorno de planificación.


#### Función `world_to_map(x_world, y_world, resolution, origin)`
Transforma coordenadas del mundo real (metros) a coordenadas discretas del mapa (celdas).

**Entradas:**
- `x_world, y_world`: coordenadas en el mundo.
- `resolution`: resolución del mapa.
- `origin`: origen del mapa.

**Salidas:**
- (x_map, y_map): coordenadas en la grilla.


#### Función `map_to_world(x_map, y_map, resolution, origin, image_height)`
Convierte coordenadas de la grilla a coordenadas del mundo real.

**Entradas:**
- `x_map, y_map`: coordenadas en la grilla.
- `resolution`: resolución del mapa.
- `origin`: origen del mapa.

image_height: altura de la imagen del mapa.

**Salidas:**
- `(x_world, y_world)`: coordenadas en el mundo real.


#### Función `filter_path_by_distance(path, target_distance, resolution)`
Filtra la trayectoria generada por Dijkstra para asegurar que los puntos consecutivos estén separados por una distancia mínima especificada en metros.

**Entradas:**
- `path`: trayectoria original en coordenadas de mapa.
- `target_distance`: distancia mínima entre puntos (m).
- `resolution`: resolución del mapa.

**Salidas:**
- `new_path`: trayectoria filtrada con separación controlada.


#### Función `save_custom_csv(path, filename, resolution, origin, image_height)`
Guarda la trayectoria final en un archivo CSV, expresando los puntos en coordenadas del mundo real.

**Entradas:**
- `path`: trayectoria en coordenadas de mapa.
- `filename`: nombre del archivo CSV de salida.
- `resolution`: resolución del mapa.
- `origin`: origen del mapa.
- `image_height`: altura del mapa.

**Salidas:**
- Archivo `.csv` con columnas `x` e `y`.


#### Bloque principal `(__main__)`
Ejecuta el flujo completo del algoritmo:
1. Carga y preprocesa el mapa.
2. Convierte los puntos de inicio y meta a coordenadas de grilla.
3. Ejecuta el algoritmo Dijkstra.
4. Filtra la trayectoria para separaciones de 0.5 m y 1.0 m.
5. Guarda las trayectorias resultantes en archivos CSV.




### 2. Algoritmo Dijkstra (`dijkstra.py`)

#### Clase `Dijkstra`
La clase `Dijkstra` hereda de la clase `AStar` y reutiliza sus estructuras básicas, desactivando el uso de heurística para realizar una búsqueda puramente basada en costo acumulado.


#### Método __init__(start, goal, env, heuristic_type)
Inicializa el planificador con los parámetros del entorno y los puntos de inicio y meta.

**Entradas:**
- `start`: coordenada inicial.
- `goal`: coordenada objetivo.
- `env`: entorno tipo Grid.
- `heuristic_type`: tipo de heurística (no utilizada en Dijkstra).

**Salidas:**
- Objeto `Dijkstra` inicializado.


#### Método `plan()`
Ejecuta el algoritmo de Dijkstra utilizando una cola de prioridad (OPEN) y una lista de nodos visitados (CLOSED). En cada iteración se expande el nodo con menor costo acumulado, se evalúan sus vecinos válidos y se detiene cuando se alcanza el nodo objetivo.

**Entradas:**
- No recibe parámetros adicionales.

**Salidas:**
- `cost`: costo total de la trayectoria.
- `path`: lista de nodos que conforman la ruta desde inicio a meta.
- `expand`: lista de nodos explorados durante la búsqueda.













## Descripcion general del algoritmo RRT:
El algoritmo Rapidly-exploring Random Tree (RRT) es un método de planificación basado en muestreo aleatorio que construye progresivamente un árbol de búsqueda desde el punto inicial hacia el espacio libre del entorno. En cada iteración, el algoritmo genera un punto aleatorio, encuentra el nodo del árbol más cercano y extiende una nueva rama en esa dirección, siempre que no exista colisión con obstáculos. El proceso se repite hasta que una rama alcanza la región cercana al objetivo o se supera el número máximo de muestras.

## Funcionamiento del algoritmo RRT en Python
### 1. Script principal: `planificador_rrt_delgado.py`
Este script ejecuta el flujo completo del planificador RRT, desde la preparación del entorno hasta la generación de archivos CSV con la trayectoria final.

#### Creación del entorno para RRT**
**Función:** `rrt_env_from_map(map_bin)`

Convierte el mapa binarizado en una estructura de tipo Map, requerida por el algoritmo RRT. Cada celda ocupada del mapa se representa como un obstáculo rectangular básico, permitiendo que el planificador realice verificación de colisiones durante la expansión del árbol.

**Entrada:**
- `map_bin`: mapa binarizado (0: libre, 1: ocupado)

**Salida:**
- `env`: entorno tipo Map compatible con RRT

#### Procesamiento y exportación de la trayectoria
Las funciones `get_spaced_waypoints` y `save_path_as_csv` cumplen el mismo propósito que en Dijkstra:
- Filtrar la trayectoria generada para imponer una separación mínima entre puntos (0.5 m y 1.0 m).
- Exportar los waypoints finales en formato CSV usando coordenadas del mundo real.

### 2. Algoritmo RRT (`rrt.py`)
#### Clase `RRT`
La clase RRT hereda de SampleSearcher y encapsula la lógica principal del algoritmo de expansión aleatoria del árbol.

**Entradas principales del planificador:**
- `start`: nodo inicial.
- `goal`: nodo objetivo.
- `env`: entorno tipo Map.
- `max_dist`: distancia máxima de expansión por iteración.
- `sample_num`: número máximo de muestras aleatorias.
- `goal_sample_rate`: probabilidad de muestrear directamente el objetivo.

#### Método `plan()`
Implementa el ciclo principal del algoritmo RRT. En cada iteración se genera un nodo aleatorio, se identifica el nodo más cercano del árbol y se intenta extender una nueva rama en esa dirección. Si la nueva rama no colisiona con obstáculos, se añade al árbol. El proceso termina cuando una rama alcanza una distancia menor que max_dist respecto al objetivo.

**Salida:**
- `cost`: costo total del camino encontrado.
- `path`: trayectoria desde el inicio hasta el objetivo.
- `expand`: conjunto de nodos generados durante la exploración.

#### Métodos auxiliares (`generateRandomNode`, `getNearest`, `extractPath`)
Estas funciones trabajan de forma conjunta para:
- Generar muestras aleatorias dentro del espacio libre.
- Encontrar el nodo más cercano en el árbol existente.
- Reconstruir la trayectoria final siguiendo los enlaces padre–hijo desde el objetivo hasta el inicio.


## Instrucciones de ejecución y visualización de resultados

Esta sección describe los pasos necesarios para clonar el repositorio, configurar el entorno y ejecutar correctamente los algoritmos de **Dijkstra** y **RRT**.

### 1. Clonar el repositorio

```bash
git clone https://github.com/JeremyD111/F1Tenth_Global_Planner_Delgado.git
cd F1Tenth_Global_Planner_Delgado
```

### 2. Crear y activar un entorno virtual 
Se recomienda utilizar un entorno virtual para evitar conflictos de dependencias:

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Instalación de dependencias

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Ejecución del algoritmo Dijkstra


```bash
cd f1tenth
python3 f1tenth_map.py
```

Durante la ejecución:

- Se cargará el mapa especificado.
- Se mostrará gráficamente el proceso de expansión del algoritmo.
- Se visualizará la trayectoria final encontrada entre el punto inicial y el objetivo.


### 5. Ejecución del algoritmo RRT

```bash
cd ~/F1Tenth_Global_Planner_Delgado
python3 planificador_rrt_delgado.py
```

**Nota:** La generacion de la trayectoria puede tardar un poco mas de 20 minutos.


### 6. Visualizacion de archivos csv con waypoints de 0.5 y 1.0

- Los archivos csv provenientes del algoritmo **Dijkstra** estaran en la carpeta `f1tenth` con los nombres de `dijkstra_0.5m.csv` y `dijkstra_1.0m.csv`


- Los archivos csv provenientes del algoritmo **RRT** estaran en la carpeta raiz con los nobres de `rrt_path_0.5m.csv` y `rrt_path_1.0m.csv`


### 7. Enlace de youtube:

https://youtu.be/slBmgillWC4





















# Parte B: Suavizado de trayectorias (Curve Generation)
## Descripción general del método de suavizado: B-Splines
Para el suavizado de trayectorias se empleó B-Splines cúbicos, utilizando la infraestructura de generación de curvas incluida en el repositorio base `python_motion_planning.utils.CurveFactory`. Se utilizó `k=3` con un paso de discretización fino de `step = 0.001`.

El algoritmo B-Spline implementado genera una curva paramétrica a partir de una secuencia de puntos mediante tres etapas principales: parametrización, construcción del vector de nudos y evaluación de funciones base. Primero, a cada punto se le asigna un parámetro escalar utilizando el método centrípeto, el cual depende de la distancia entre puntos consecutivos y ayuda a evitar oscilaciones en regiones con cambios bruscos. Luego, se construye un vector de nudos normalizado que define los intervalos donde actúan las funciones base. A partir de estos parámetros, se calculan las funciones base B-Spline usando la formulación recursiva de Cox–de Boor, que pondera localmente los puntos de control. Finalmente, la curva se evalúa como una combinación lineal de dichas funciones base con los puntos de control, produciendo una trayectoria continua donde cada segmento depende solo de un subconjunto de puntos vecinos, suavizando la ruta original sin alterar globalmente su forma.

## Funcionamiento del algoritmo B-Splines en Python

### 1. Script principal para Dijkstra: suavizar_trayectorias.py
Este script toma como entrada los archivos CSV generados por Dijkstra y produce trayectorias suavizadas, además de su visualización y exportación.

#### Procesamiento de la trayectoria

**Función:** `procesar_trayectoria(archivo_csv, factor_filtrado, curve_factory)`
Carga una trayectoria discreta desde un archivo CSV, reduce el número de puntos para evitar sobreajuste y genera una curva suave mediante B-Spline.

**Entradas:**
- `archivo_csv`: archivo con waypoints originales.
- `factor_filtrado`: factor de submuestreo de puntos.
- `curve_factory`: fábrica de curvas del repositorio.

**Salidas:**
- Trayectoria original.
- Puntos filtrados (usados como control).
- Trayectoria suavizada (lista de puntos).

#### Guardado y visualización

Las funciones `guardar_csv` y `configurar_plot_completo` se encargan, respectivamente, de:
- Exportar la trayectoria suavizada a un nuevo archivo CSV.
- Mostrar de forma comparativa los waypoints originales, los puntos de control y la curva B-Spline resultante.


### 2. Algoritmo de suavizado: B-Spline (`bspline_curve.py`)
#### Clase `BSpline`
Implementa la generación de curvas B-Spline a partir de un conjunto de puntos de entrada. El algoritmo no interpola directamente cada punto del camino, sino que calcula una combinación ponderada de funciones base que definen una curva suave global.

**Entradas principales:**
- `step`: resolución de muestreo de la curva.
- `k`: grado de la B-Spline.
- `points`: puntos de control de la trayectoria.

**Salida:**
- Lista ordenada de puntos que representan la trayectoria suavizada.


### 3. Script principal para RRT: `suavizar_trayectorias_rrt.py`
Este script implementa el postprocesamiento de trayectorias generadas por el algoritmo RRT, aplicando el mismo método de suavizado mediante B-Splines utilizado previamente para Dijkstra. Por esta razón, en esta sección se describe únicamente el flujo y las funciones propias del procesamiento de trayectorias RRT, sin repetir el funcionamiento interno del algoritmo B-Spline.

#### Función `procesar_trayectoria`
Carga una trayectoria generada por RRT desde un archivo CSV, corrige problemas típicos del algoritmo (como puntos duplicados consecutivos), filtra los puntos de control y genera una trayectoria suavizada usando B-Spline.

**Entradas:**
- `archivo_csv`: archivo CSV con los waypoints generados por RRT.
- `factor_filtrado`: entero que define el submuestreo de puntos.
- `curve_factory`: fábrica de curvas del módulo python_motion_planning.

**Salidas:**
- `puntos_crudos`: trayectoria original generada por RRT.
- `puntos_filtrados`: puntos de control usados para el suavizado.
- `trayectoria_suave`: trayectoria continua generada por B-Spline.

#### Función `guardar_csv`
Esta función exporta la trayectoria suavizada a un archivo CSV con formato (x, y).

#### Función `configurar_plot_completo`
Se encarga de la visualización estática de los resultados, mostrando simultáneamente:
- Los waypoints originales de RRT.
- Los puntos de control usados para el suavizado.
- La curva B-Spline completa.


## Instrucciones de ejecución

### Suavizar trayectoria generada por Dijkstra

Desde la raíz del repositorio

```bash
cd ~/F1Tenth_Global_Planner_Delgado
python3 suavizar_trayectorias.py
```
Veremos como se genera simultaneamente dos animaciones donde se suavizan las trayectorias con waypoints de 0.5m y 1.0m respectivamente 

### Suavizar trayectoria generada por RRT

```bash
cd ~/F1Tenth_Global_Planner_Delgado
python3 suavizar_trayectorias_rrt.py
```
NUevamente veremos como se generan dos animaciones donde se suavizan las trayectorias con waypoints de 0.5m y 1.0m respectivamente

### Visualizacion de archivos csv con waypoints de 0.5 y 1.0

- Los archivos csv de la curva suavizada del algoritmo **Dijkstra** estaran en la carpeta `f1tenth` con los nobres de `dijkstra_0.5m_smooth.csv` y `dijkstra_1.0m_smooth.csv`


- Los archivos csv de la curva suavizada del algoritmo **RRT** estaran en la carpeta raiz con los nobres de `rrt_path_0.5m_smooth.csv` y `rrt_path_1.0m_smooth.csv`


### Enlace de youtube:

https://www.youtube.com/watch?v=cH7kaLIMNUA 







