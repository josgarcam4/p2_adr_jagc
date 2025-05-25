# Práctica 2: Filtro de Kalman en ROS 2
Este repositorio contiene el código implementado para la **Práctica 2: Filtro de Kalman (KF)** de la asignatura de *Ampliación de Robótica*, desarrollado por el alumno José Antonio García Campanario.

## Estructura del repositorio
 - `kalman_filter.py`: Implementación del filtro de Kalman con configuraciones de ruido ajustables (bajo y alto).
 - `kf_estimation.py`: Nodo que implementa el filtro de Kalman para estimar posición y orientación.
 - `kf_estimation_vel.py`: Nodo que implementa el filtro de Kalman extendido para estimar posición, orientación y velocidades.
 - `motion_models.py`: Modelos de movimiento (matrices A y B).
 - `observation_models.py`: Modelos de observación (matriz C).
 - `sensor_utils.py`: Funciones auxiliares para trabajar con sensores y simulaciones.
 - `visualization.py`: Herramientas para visualizar los resultados.

## Diseño del filtro
Los filtros han sido diseñados siguiendo las especificaciones propuestas por el profesorado.

### Kalman Filter 1 (KF1): 
Este filtro utiliza los estados de posición y orientación en 2D (`x`, `y`, `theta`), junto con una matriz de transición `A` y una matriz de control `B`. El flujo de trabajo en el nodo `kf_estimation` es el siguiente:
- **Predicción**: En cada mensaje de odometría recibido, el filtro predice el nuevo estado del robot utilizando el modelo de movimiento y el último comando de velocidad (`self.u`). Esto genera una estimación previa de la posición y la incertidumbre asociada.
- **Actualización**: Se genera una observación simulada de la posición real (con posible ruido o deriva) utilizando la función `self.odom_simulator.add_drift`. El filtro compara esta observación con la predicción y ajusta la estimación del estado y la incertidumbre, combinando ambas fuentes de información según su confianza (ruido).
- **Publicación**: Finalmente, el nodo publica tanto la estimación del filtro (incluyendo su covarianza) como la posición real simulada, para facilitar la comparación y visualización de los resultados.

### Kalman Filter 2 (KF2):
Este filtro extiende el modelo anterior para incluir las velocidades lineales y angulares (`vx`, `vy`, `omega`), además de la posición y orientación (`x`, `y`, `theta`). En este caso, la matriz de transición `A` no depende directamente del control. El flujo de trabajo en el nodo `kf_estimation_vel` es el siguiente:
- **Predicción**: En cada ciclo, el filtro predice la evolución de todas las variables del estado, incluidas las velocidades, utilizando el modelo de movimiento y el control recibido (velocidad lineal y angular).
- **Actualización**: Cuando se recibe una nueva observación simulada (con ruido) mediante la función `generate_noisy_measurement_2`, el filtro actualiza todas las variables del estado, corrigiendo tanto la posición como las velocidades según la información sensorial y la confianza definida.
- **Publicación**: Finalmente, el nodo publica tanto la estimación del filtro (incluyendo su covarianza) como la posición real simulada, para facilitar la comparación y visualización de los resultados.

El filtro de Kalman 2 permite una estimación más completa y precisa, integrando tanto la posición como las velocidades en un único modelo.

## Configuración de ruido bajo
Se han utilizado los siguientes valores como "ruido bajo":
- **KF1**: `[0.03, 0.03, 0.02]` para `[x, y, theta]`.
- **KF2**: `[0.03, 0.03, 0.02, 0.03, 0.03, 0.02]` para `[x, y, theta, vx, vy, omega]`.

En esta configuración, ambos filtros ofrecen estimaciones razonablemente precisas. Sin embargo, el filtro de Kalman 1 puede presentar ligeras oscilaciones y una deriva acumulativa con el tiempo, mientras que el filtro de Kalman 2 genera trayectorias más suaves y cercanas a la realidad.

## Configuración de ruido alto solo en la medición (Q)
Se han utilizado los siguientes valores como "ruido alto" para la medición:
- **KF1**: `[0.06, 0.06, 0.04]` para `[x, y, theta]`.
- **KF2**: `[0.06, 0.06, 0.04, 0.06, 0.06, 0.04]` para `[x, y, theta, vx, vy, omega]`.

Con esta configuración:
- **KF1**: La trayectoria estimada se vuelve más errática y presenta mayores oscilaciones debido a la menor confianza en las mediciones.
- **KF2**: Aunque también se ve afectado, el filtro 2 mantiene una mayor estabilidad gracias a la integración de las velocidades en el modelo.

## Configuración de ruido alto solo en el proceso (R)
Se han utilizado los siguientes valores como "ruido alto" para el proceso:
- **KF1**: `[0.06, 0.06, 0.04]` para `[x, y, theta]`.
- **KF2**: `[0.06, 0.06, 0.04, 0.06, 0.06, 0.04]` para `[x, y, theta, vx, vy, omega]`.

Con esta configuración:
- **KF1**: La estimación se vuelve más dispersa y menos precisa, ya que el filtro confía menos en su modelo de movimiento.
- **KF2**: El filtro 2 sigue siendo más robusto y estable, compensando mejor la incertidumbre en el modelo de movimiento gracias a la estimación conjunta de velocidades.

## Limitaciones
No se han podido generar las gráficas de las trayectorias debido a problemas con Gazebo, lo que impidió ejecutar el entorno de simulación necesario para el lanzamiento del robot.

## Ejecución de los nodos
La ejecución del código mantiene lo propuesto anteriormente, siendo necesario realizar los siguientes pasos:
- git clone https://github.com/josgarcam4/p2_adr_jagc.git
- cd p2_adr_jagc
- colcon build --packages-select p2_adr_jagc
- source install/setup.bash

A continuación hace falta tener un terminal abierto con el robot:
- ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true

Finalmente, seleccionar cual de los dos filtros se va a utilizar en una nueva ventana. Adicionalmente, para modificar el ruido usado, es necesario entrar en el archivo /filters/kalman_filter.py y cambiar los valores seleccionados como low por high. La ejecución de los mismos se realiza mediante la ejecución de el siguiente código en función de si se precede usar velocidad o posición:
- ros2 run p2_adr_jagc kf_estimation
- ros2 run p2_adr_jagc kf_estimation_vel
