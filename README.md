# Práctica 2: Filtro de Kalman en ROS 2

En esta práctica se implementará un Filtro de Kalman (KF) para estimar la posición y velocidad de un robot móvil simulado con ROS 2. El objetivo principal es desarrollar un filtro de Kalman puro (lineal), utilizando únicamente modelos lineales de movimiento y observación. Para ello, se han estudiado tres diferentes configuraciones de ruido.

---

## Estructura del repositorio

### Estructura del Filtro
Para el estudio del Filtro del Kalman se implementaron dos versiones:
1. **KalmanFilter**: Modelo básico que estima solo posición/orientación [x, y, θ], cuyo código se halla en la carpeta `/p2_kf_pdb/filters/kalman_filter.py`
2. **KalmanFilter_2**: Modelo extendido que estima posición/orientación y velocidad [x, y, θ, vx, vy, ω], cuyo código se halla en la carpeta `/p2_kf_pbd/filters/kalman_filter.py`

### Modelos y Nodos
A continuación, se indicará el contenido de los scripts que contiene el paquete, `/p2_kf_pbd/~`:
1. **kf_estimation.py**: Nodo con el modelo básico (posición)
2. **kf_estimation_vel.py**: Nodo con el modelo extendido (posición y velocidad) 
3. **motion_models.py**: Modelos de movimiento (predicción).
   - Para el modelo básico: Matriz de transición A y matriz de control B.
   - Para el modelo extendido: Incluye términos para la velocidad lineal y angular
5. **observation_models.py**: Modelo de observación (corrección). C
6. **sensor_utils.py**: Funciones de alto nivel para los sensores
7. **visualization.py**: Funciones de visualización de resultados


Este repositorio contiene el código base para la **Práctica 2** de la asignatura de *Ampliación de Robótica*, cuyo objetivo es implementar un **Filtro de Kalman (KF)** en un entorno simulado con **ROS 2**.

El ejercicio se divide en dos partes: una primera aproximación basada en odometría con estimación de posición, y una segunda con estimación de posición y velocidad utilizando un modelo de estado extendido.

---

## Estructura del repositorio
 - kalman_filter.py # Implementación del KF con TODOs para completar 
 - kf_estimation.py # Nodo con el modelo básico de KF (posición)
 - kf_estimation_vel.py # Nodo con el modelo completo de KF (posición y velocidad) 
 - motion_models.py # Modelos de movimiento A y B 
 - observation_models.py # Modelos de observación C
 - sensor_utils.py # Funciones de alto nivel para facilitar las cosas con los sensores
 - visualization.py # Funciones de visualización de resultados
 

## Instrucciones

### Requisitos previos
Descargar el simulador y los paquetes dependientes del mismo para poder trabajar con el robot Turtlebot 4:

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-dev-tools

```

### 1. Clonar el repositorio
Fuera del docker (en tu ubuntu o en el WSL)

```bash
mkdir -p ~/AdR/p2_ws/src
cd p2_ws/src
git clone https://github.com/miggilcas/p2_kf_adr
cd p2_kf_adr
```
### 2. Construir el paquete
Ya dentro del Docker:
```bash
cd ~/AdR/p2_ws
colcon build --packages-select p2_kf_adr
source install/setup.zsh  # o setup.bash si no estás usando el docker
```
### 3. Lanzar el simulador
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### 4. Ejecutar el nodo del filtro de Kalman
#### Modelo 1: estimación de posición
```bash
ros2 run p2_kf_adr kf_estimation
```
#### Modelo 2:
```bash
ros2 run p2_kf_adr kf_estimation_vel
```

## Objetivo de la práctica

- Comprender y programar un filtro de Kalman básico para estimar la posición del robot.
- Ampliar el modelo de estado para incluir velocidad y emplear un modelo lineal puro.
- Comparar el comportamiento del filtro con diferentes configuraciones de ruido.
- Preparar el terreno para el uso de un Filtro de Kalman Extendido (EKF) en la siguiente práctica.

## Qué deben completar los estudiantes
Los archivos kalman_filter.py, motion_models.py y observation_models.py contienen TODOs que los alumnos deben implementar.

Las clases principales son:

- KalmanFilter – Para el modelo simple (posición).
- KalmanFilter_2 – Para el modelo completo (posición + velocidad).

## Entrega
Los estudiantes deberán subir a GitHub o entregar un archivo .zip con nombre: p2_kf_<iniciales> (por ejemplo: p2_kf_mgc).

El repositorio o archivo.zip debe contener:

1. Código completo con los TODOs resueltos.

2. Capturas o gráficas de los resultados de estimación para ambos modelos.

3. Experimentos con tres configuraciones distintas:
    - Ruido bajo.
    - Ruido alto en la medida.
    - Ruido alto en el proceso.

4. Un README o una pequeña memoria en PDF explicando:
    - Cómo se ha implementado cada parte.
    - Resultados observados en los tres casos.
    - Breve análisis de por qué ocurre lo observado.

## Comentarios adicionales
Podéis cambiarle el nombre al paquete y ponerle el mismo que a la entrega, pero sed consistentes a la hora de configurar el paquete y que esté ese nombre en todos lados para que compile bien (tanto en el nombre de la carpeta donde estarán los scripts como en el setup.cfg, como en el setup.py y como en el package.xml).
