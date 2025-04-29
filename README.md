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

## Métodos `predict` y `update` de las clases `KalmanFilter` y `KalmanFilter_2`

### `KalmanFilter`

#### `predict(u, dt)`
Realiza la predicción del estado del sistema utilizando el modelo de movimiento.

```python
kf = KalmanFilter(initial_state, initial_covariance)
mu, Sigma = kf.predict(u, dt)
```
## Resultados y discusión de las gráficas 



4. Un README o una pequeña memoria en PDF explicando:
    - Cómo se ha implementado cada parte.
    - Resultados observados en los tres casos.
    - Breve análisis de por qué ocurre lo observado.


