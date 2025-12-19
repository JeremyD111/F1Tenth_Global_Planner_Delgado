# Global Path Planning – RRT y Dijkstra aplicado a F1TENTH

Este repositorio contiene la implementación y documentación de algoritmos de **planificación global de trayectorias**, desarrollados como parte de una tarea académica del curso de robótica.

El trabajo se basa en el repositorio original:

**Global_Planner**  
https://github.com/widegonz/Global_Planner

A partir de este repositorio base, se realizaron adaptaciones, extensiones y modificaciones para:

- Implementar y analizar el algoritmo **RRT (Rapidly-exploring Random Tree)**.
- Ejecutar y comparar el algoritmo **Dijkstra** sobre mapas reales.
- Adaptar ambos algoritmos para trabajar con mapas del entorno **F1TENTH**.
- Generar trayectorias y waypoints exportables para navegación.

El objetivo principal del repositorio es que cualquier usuario, siguiendo las instrucciones aquí descritas, pueda **clonar el repositorio, ejecutar los algoritmos y reproducir los resultados obtenidos**, sin necesidad de configuraciones adicionales.


## Descripción del algoritmo asignado: Dijkstra

El algoritmo de **Dijkstra** es un método clásico de planificación de rutas que permite encontrar el **camino de costo mínimo** entre un nodo inicial y todos los demás nodos de un grafo ponderado, siempre que los costos de las aristas sean no negativos.

En el contexto de planificación de trayectorias robóticas, Dijkstra se utiliza para buscar la **ruta óptima** entre un punto inicial y un punto objetivo sobre una representación discreta del entorno, generalmente modelada como una grilla ocupacional. Cada celda del mapa corresponde a un nodo del grafo, y las transiciones entre celdas adyacentes representan las aristas del grafo.

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

Al finalizar, el algoritmo garantiza que la trayectoria obtenida corresponde al **camino de menor costo global** entre el inicio y el objetivo.

### Características principales

- Garantiza optimalidad del camino encontrado.
- Es determinista y reproducible.
- Su costo computacional aumenta significativamente en mapas grandes o de alta resolución.
- Resulta adecuado como referencia para comparar algoritmos más eficientes o probabilísticos.

En este repositorio, el algoritmo de Dijkstra se utiliza como **método base de planificación global**, permitiendo comparar su desempeño y características frente al algoritmo RRT.


## Instrucciones de ejecución y visualización de resultados

Esta sección describe los pasos necesarios para clonar el repositorio, configurar el entorno y ejecutar correctamente los algoritmos de **Dijkstra** y **RRT**, de forma que los resultados puedan ser reproducidos sin inconvenientes.

### 1. Clonar el repositorio

Primero, clonar el repositorio desde GitHub:

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
pip install -r requirements.txt
```

### 4. Ejecución del algoritmo Dijkstra

Desde el directorio principal del repositorio, ejecutar:

```bash
python3 global_examples.py
```

Durante la ejecución:

- Se cargará el mapa especificado.
- Se mostrará gráficamente el proceso de expansión del algoritmo.
- Se visualizará la trayectoria final encontrada entre el punto inicial y el objetivo.

El resultado corresponde al camino óptimo global calculado mediante Dijkstra sobre el mapa discreto.


### 5. Ejecución del algoritmo RRT

```bash
python3 planificador_rrt_delgado.py
```

































