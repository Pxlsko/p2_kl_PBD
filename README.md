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
3. **motion_models.py**: Modelos de movimiento (predicción)
   - Para el modelo básico: Matriz de transición A y matriz de control B.
   - Para el modelo extendido: Además de lo anterior mencionado, incluye los términos para la velocidad lineal y angular
5. **observation_models.py**: Modelo de observación (corrección)
   - Matrices C para mapear el estado interno a las observaciones medidas 
7. **sensor_utils.py**: Funciones de alto nivel para los sensores
8. **visualization.py**: Funciones de visualización de resultados

## Implementación para cada caso 

### Filtro de Kalman (Modelo Básico)

#### Función de predicción: `predict(u, dt)'
Este método realiza la predicción del estado del sistema utilizando el modelo de movimiento. Calcula el nuevo estado estimado (mu) y la incertidumbre asociada (Sigma) en función de las matrices de transición de estado (A) y de entrada de control (B), el vector de control u, y el intervalo de tiempo dt. También incorpora la covarianza del ruido del proceso (R).

#### Función de actualización: `update(z)`
Este método actualiza el estado estimado del sistema utilizando una observación (z). Calcula la ganancia de Kalman (K), ajusta el estado estimado (mu) en función de la diferencia entre la observación y la predicción, y actualiza la incertidumbre (Sigma) considerando el modelo de observación (C) y la covarianza del ruido de observación (Q).

### Filtro de Kalman (Modelo Extendido)

#### Función de predicción: `predict(u=None, dt=1.0)'
Este método realiza la predicción del estado extendido del sistema (que incluye velocidades) utilizando el modelo de movimiento. Calcula el nuevo estado estimado (mu) y la incertidumbre asociada (Sigma) en función de la matriz de transición de estado (A), el intervalo de tiempo dt, y la covarianza del ruido del proceso (R). El vector de control u es opcional.

#### Función de actualización: `update(z)`
Este método actualiza el estado extendido del sistema utilizando una observación (z). Calcula la ganancia de Kalman (K), ajusta el estado estimado (mu) en función de la observación y actualiza la incertidumbre (Sigma) considerando el modelo de observación (C) y la covarianza del ruido de observación (Q).

### Estimación del filtro (Modelo Básico)

#### Función de odometría: `odom_callback`
Este método procesa los datos de odometría recibidos. Inicializa la posición inicial si no está definida, calcula el estado actual del sistema en función de la odometría y el tiempo transcurrido, y utiliza el filtro de Kalman para predecir y actualizar el estado estimado. También gestiona la visualización y la publicación de los estados estimados y reales.

#### Publicador de posición estimada: `publish_estimated_pose`
Publica la posición estimada del sistema, incluyendo la covarianza asociada, en un mensaje de tipo PoseWithCovarianceStamped. Este mensaje contiene la posición, orientación y la incertidumbre del estado estimado.

#### Publicador de posición actual : `publish_real_pose`
Publica la posición real del sistema en un mensaje de tipo PoseWithCovarianceStamped. Este mensaje incluye la posición y orientación reales, pero no considera la covarianza del estado.

### Estimación del filtro (Modelo Extendido)

#### Función de odometría: `odom_callback`
Este método procesa los datos de odometría recibidos. Inicializa el estado inicial del filtro de Kalman extendido si no está definido, calcula el estado actual del sistema en función de la odometría y el tiempo transcurrido, y utiliza el filtro para predecir y actualizar el estado estimado. También gestiona la visualización y la publicación de los estados estimados y reales.

#### Publicador de posición estimada: `publish_estimated_pose`
Publica la posición estimada del sistema, incluyendo la covarianza asociada, en un mensaje de tipo PoseWithCovarianceStamped. Este mensaje contiene la posición, orientación y la incertidumbre del estado estimado.

#### Publicador de posición actual : `publish_real_pose`
Publica la posición real del sistema en un mensaje de tipo PoseWithCovarianceStamped. Este mensaje incluye la posición y orientación reales, pero no considera la covarianza del estado.

## Resultados y discusión de las gráficas 



