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

Este filtro utiliza los estados de posición y orientación en dos dimensiones (x, y, θ), junto con la matriz de transición de estados *A* y una matriz de control *B*. El proceso de predicción y estimación se lleva a cabo en el módulo de estimación del filtro de Kalman de la siguiente manera:

- Predicción: En cada mensaje de odometría recibido, el filtro de Kalman comienza prediciendo el nuevo estado del robot utilizando el modelo de movimiento y el último comando de velocidad `self.u`. Este proceso genera una estimación preliminar de la posición y la incertidumbre asociada con el estado del robot.

- Actualización: Posteriormente, se genera una observación simulada de la posición real del robot, que puede incluir ciertos niveles de deriva o ruido, mediante la función `self.odom_simulator.add_drift`. El filtro de Kalman compara esta observación con la predicción inicial y ajusta la estimación del estado, así como la incertidumbre asociada, combinando ambas fuentes de información según la confianza de cada una, determinada por el ruido.

- Publicación: Finalmente, el nodo publica tanto la estimación obtenida por el filtro de Kalman, junto con su covarianza, como la posición real simulada. Esto permite realizar comparaciones y visualizar los resultados de la estimación y la observación real del sistema.

### Filtro de Kalman (Modelo Extendido)

Este filtro emplea los estados de posición, orientación, y las velocidades lineales y angulares (x, y, θ, vx, vy, 𝜔), junto con la matriz de transición de estados *A*. El sistema está diseñado para ser un modelo puro, sin control directo, lo que implica que no se incluye un control explícito de las variables del sistema. El proceso de predicción y estimación se lleva a cabo en el módulo de estimación de velocidad del filtro de Kalman de la siguiente forma:

- Predicción: En cada ciclo, el filtro predice la evolución de todas las variables del estado, incluidas las velocidades, utilizando el modelo de movimiento y el control recibido (velocidades lineales y angulares). Este proceso proporciona una estimación anticipada de la posición y las velocidades, junto con la incertidumbre asociada.

- Actualización: Cuando se recibe una nueva observación simulada, que incluye ruido, mediante la función `generate_noisy_measurement_2`, el filtro actualiza todas las variables del estado. Esto implica corregir tanto la posición como las velocidades, ajustando las estimaciones con la información sensorial disponible y teniendo en cuenta la confianza de cada fuente de información, determinada por el nivel de ruido presente.

- Publicación: Finalmente, el nodo publica tanto la estimación del filtro de Kalman, junto con su covarianza, como la posición real simulada. Esto permite realizar comparaciones y visualizar los resultados, proporcionando una referencia entre la estimación y la observación real.

El filtro de Kalman 2 ofrece una estimación conjunta y coherente de la posición y las velocidades del robot, mejorando así la precisión y la utilidad de la información para su navegación y control.

## Resultados y discusión de las gráficas 
Como aclaración, debido a que no poseo partición de disco para Ubuntu (no por ganas si no porque mi ordenador me lo impide), los resultados se han observado desde la CMD del Docker. Por tanto, los resultados no serán tan visibles, desgraciadamente.

### Ruido Bajo
El ruido del cual se ha hecho uso, para los modelos básico (KF1) y extendido (KF2), han sido `[0.02, 0.02, 0.01]` y `[0.02, 0.02, 0.01, 0.02, 0.02, 0.01]`, respectivamente. Los resultados de los modelos pueden verse en el directorio `Images/LowNoise_kf1.png` y `Images/LowNoise_kf2.png`, respectivamente.



high_noise_std_KF1 = [0.1, 0.1, 0.03]
high_noise_std_KF2 = [0.1, 0.1, 0.03, 0.1, 0.1, 0.03]

### Ruido Alto en la medición

### Ruido Alto en el proceso





