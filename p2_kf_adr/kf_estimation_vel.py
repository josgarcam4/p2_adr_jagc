import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import numpy as np

# Importación de módulos personalizados
from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

class KalmanFilterPureNode(Node):
    """
    Nodo de ROS 2 que utiliza un filtro de Kalman para estimar el estado de un robot móvil
    basado en datos de odometría.
    """

    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # Inicialización del estado y la covarianza inicial del filtro de Kalman
        initial_state = np.zeros(6)  # Estado inicial: [x, y, theta, vx, vy, omega]
        initial_covariance = np.eye(6) * 0.1  # Baja incertidumbre inicial

        # Instancia del filtro de Kalman
        self.kf = KalmanFilter_2(initial_state, initial_covariance)

        # Visualizador para mostrar resultados
        self.visualizer = Visualizer()

        # Variables internas
        self.initial_pose = None  # Pose inicial del robot
        self.first_prediction_done = False  # Indica si se ha realizado la primera predicción
        self.prev_time = None  # Tiempo de la última medición
        self.u = np.zeros(2)  # Vector de control [vx, omega]

        # Suscripción al tópico de odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Tópico de entrada
            self.odom_callback,  # Callback para procesar los datos
            10  # Tamaño de la cola
        )

        # Publicación de la estimación del filtro de Kalman
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',  # Tópico de salida
            10  # Tamaño de la cola
        )

    def odom_callback(self, msg):
        """
        Callback que procesa los datos de odometría y realiza los pasos del filtro de Kalman.
        """
        # Convierte la odometría en una pose 2D (x, y, theta)
        current_pose = odom_to_pose2D(msg)

        # Extrae las velocidades lineales y angulares del mensaje de odometría
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        vx = linear.x  # Velocidad lineal en x
        vy = linear.y  # Velocidad lineal en y
        omega = angular.z  # Velocidad angular

        # Si es la primera medición, inicializa la pose inicial y el estado del filtro
        if self.initial_pose is None:
            self.initial_pose = current_pose
            self.kf.mu = np.array(list(self.initial_pose) + [0, 0, 0])  # [x, y, theta, vx, vy, omega]
            self.prev_time = self.get_clock().now().nanoseconds  # Guarda el tiempo actual
            return

        # Calcula la pose relativa normalizada respecto a la pose inicial
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        # Actualiza el vector de control [vx, omega]
        self.u = np.array([vx, omega])

        # Calcula el intervalo de tiempo (dt) desde la última medición
        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time is not None:
            dt = (curr_time - self.prev_time) / 1e9  # Convierte nanosegundos a segundos
        else:
            dt = 0.0
        self.prev_time = curr_time  # Actualiza el tiempo previo

        # Paso de predicción del filtro de Kalman
        mu, sigma = self.kf.predict(self.u, dt)
        self.first_prediction_done = True

        # Paso de actualización del filtro de Kalman
        if self.first_prediction_done:
            # Extrae el estado estimado actual
            x, y, theta, vx, vy, omega = self.kf.mu

            # Genera una medición ruidosa basada en la pose actual y las velocidades
            z = generate_noisy_measurement_2(current_pose, vx, vy, omega)

            # Actualiza el filtro de Kalman con la medición ruidosa
            mu, sigma = self.kf.update(z)

            # Actualiza la visualización con los datos actuales
            self.visualizer.update(self.normalized_pose, mu, sigma, step="update")

            # Publica la estimación del filtro de Kalman
            self.publish_estimated_pose(mu, sigma)

            # Publica la pose real normalizada
            self.publish_real_pose(self.normalized_pose)

            # Registra información en el log
            self.get_logger().info(f"Estado actualizado: {mu},{sigma}")
            self.get_logger().info(f"Posición actualizada: {self.normalized_pose}")

    def publish_estimated_pose(self, mu, sigma):
        """
        Publica la estimación del filtro de Kalman en el tópico /kf2_estimate.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Asigna la posición estimada
        msg.pose.pose.position.x = mu[0]
        msg.pose.pose.position.y = mu[1]
        msg.pose.pose.position.z = 0.0

        # Asigna la orientación estimada (convertida a cuaterniones)
        msg.pose.pose.orientation.z = np.sin(mu[2] / 2.0)
        msg.pose.pose.orientation.w = np.cos(mu[2] / 2.0)

        # Asigna la covarianza estimada
        for i in range(3):
            for j in range(3):
                msg.pose.covariance[i * 6 + j] = sigma[i, j]

        # Publica el mensaje
        self.publisher.publish(msg)

    def publish_real_pose(self, pose):
        """
        Publica la pose real (normalizada) en el mismo formato que la estimación.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Asigna la posición real
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.0

        # Asigna la orientación real (convertida a cuaterniones)
        msg.pose.pose.orientation.z = np.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = np.cos(pose[2] / 2.0)

        # Publica el mensaje
        self.publisher.publish(msg)

def main(args=None):
    """
    Función principal que inicializa y ejecuta el nodo de ROS 2.
    """
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)  # Mantiene el nodo en ejecución
    rclpy.shutdown()  # Finaliza ROS 2 al terminar

