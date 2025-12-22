# Parte A: Global Path Planning – RRT y Dijkstra aplicado a F1TENTH

El trabajo se basa en el repositorio original:

**Global_Planner:** https://github.com/widegonz/Global_Planner

A partir de este repositorio base, se realizaron adaptaciones y modificaciones para:

- Implementar el algoritmo **RRT (Rapidly-exploring Random Tree)** en el mapa BrandsHatch.
- Implementar el algoritmo **Dijkstra** en el mapa BrandsHatch.
- Generar trayectorias y waypoints exportables para navegación.


## Descripción del algoritmo asignado: Dijkstra

El algoritmo de **Dijkstra** es un método clásico de planificación de rutas que permite encontrar el camino de costo mínimo entre un nodo inicial y todos los demás nodos de un grafo ponderado, siempre que los costos de las aristas sean no negativos.

Dijkstra se utiliza para buscar la **ruta óptima** entre un punto inicial y un punto objetivo sobre una representación discreta del entorno, generalmente modelada como una grilla ocupacional. Cada celda del mapa corresponde a un nodo del grafo, y las transiciones entre celdas adyacentes representan las aristas del grafo.

### Representación del entorno
Para este trabajo, el entorno se obtiene a partir de un mapa en formato `.yaml` y una imagen asociada que describe los obstáculos. El mapa es procesado para generar una grilla binaria, donde:

- Las celdas libres representan nodos transitables.
- Las celdas ocupadas representan obstáculos y no son consideradas durante la búsqueda.

Cada nodo se conecta con sus vecinos inmediatos (4 u 8 conectividades, dependiendo de la configuración del planificador), y a cada transición se le asigna un costo asociado a la distancia recorrida.

### Funcionamiento del algoritmo
El algoritmo de Dijkstra opera de la siguiente manera:

1. Se inicializa el nodo de inicio con un costo acumulado igual a cero.
2. Se mantiene una estructura de datos (cola de prioridad) que siempre selecciona el nodo con el menor costo acumulado.
3. En cada iteración, se expande el nodo con menor costo y se actualizan los costos de sus vecinos si se encuentra una ruta más corta.
4. El proceso continúa hasta que el nodo objetivo es alcanzado o se hayan evaluado todos los nodos accesibles.

Al finalizar, el algoritmo garantiza que la trayectoria obtenida corresponde al camino de menor costo global entre el inicio y el objetivo.

### Características principales
- Garantiza optimalidad del camino encontrado.
- Es determinista y reproducible.
- Su costo computacional aumenta significativamente en mapas grandes o de alta resolución.


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

El resultado corresponde al camino óptimo global calculado mediante Dijkstra sobre el mapa discreto.


### 5. Ejecución del algoritmo RRT

```bash
cd ~/F1Tenth_Global_Planner_Delgado
python3 planificador_rrt_delgado.py
```

**NOta:** La generacion de la trayectoria puede tardar un poco mas de 20 minutos.


### 6. Visualizacion de archivos csv con waypoints de 0.5 y 1.0

- Los archivos csv provenientes del algoritmo **Dijkstra** estaran en la carpeta `f1tenth` con los nobres de `dijkstra_0.5m.csv` y `dijkstra_1.0m.csv`


- Los archivos csv provenientes del algoritmo **RRT** estaran en la carpeta raiz con los nobres de `rrt_path_0.5m.csv` y `rrt_path_1.0m.csv`


### 7. Enlace de youtube:

https://youtu.be/slBmgillWC4


# Parte B: Suavizado de trayectorias (Curve Generation)

Una vez obtenida la trayectoria discreta a partir de los algoritmos de planificación global (Dijkstra y RRT), se aplica un proceso de suavizado de trayectoria (path smoothing) con el objetivo de generar rutas más continuas, suaves y físicamente realizables para un vehículo de **F1TENTH**.


## Descripción del método de suavizado: B-Splines

Para el suavizado de trayectorias se empleó B-Splines cúbicos, utilizando la infraestructura de generación de curvas incluida en el repositorio base `python_motion_planning.utils.CurveFactory`. Se utilizó `k=3` con un paso de discretización fino de `step = 0.001`.

El algoritmo B-Spline implementado genera una curva paramétrica a partir de una secuencia de puntos mediante tres etapas principales: parametrización, construcción del vector de nudos y evaluación de funciones base. Primero, a cada punto se le asigna un parámetro escalar utilizando el método centrípeto, el cual depende de la distancia entre puntos consecutivos y ayuda a evitar oscilaciones en regiones con cambios bruscos. Luego, se construye un vector de nudos normalizado que define los intervalos donde actúan las funciones base. A partir de estos parámetros, se calculan las funciones base B-Spline usando la formulación recursiva de Cox–de Boor, que pondera localmente los puntos de control. Finalmente, la curva se evalúa como una combinación lineal de dichas funciones base con los puntos de control, produciendo una trayectoria continua donde cada segmento depende solo de un subconjunto de puntos vecinos, suavizando la ruta original sin alterar globalmente su forma.


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


















