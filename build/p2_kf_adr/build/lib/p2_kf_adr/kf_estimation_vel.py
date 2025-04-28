import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from kalman_filter import KalmanFilter_2

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def odom_callback(self, msg):
        # Extraer posición, orientación y velocidades del mensaje de odometría
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        # Convertir orientación de cuaternión a ángulo theta
        theta = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                           1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        # Crear el vector de observación z = [x, y, theta, vx, vy, omega]
        z = np.array([
            position.x,
            position.y,
            theta,
            linear_velocity.x,
            linear_velocity.y,
            angular_velocity.z
        ])

        # Calcular el intervalo de tiempo (dt)
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convertir a segundos
        if not hasattr(self, 'last_time'):
            self.last_time = current_time  # Inicializar el tiempo anterior
        dt = current_time - self.last_time
        self.last_time = current_time

        # Crear el vector de control u (si tienes aceleraciones, inclúyelas aquí)
        u = np.zeros(3)  # Por defecto, sin control input

        # Ejecutar el paso de predicción
        self.kf.predict(u=u, dt=dt)

        # Ejecutar el paso de actualización
        self.kf.update(z=z)

        # Publicar el estado estimado
        estimated_state = self.kf.mu
        estimated_covariance = self.kf.Sigma

        # Crear un mensaje PoseWithCovarianceStamped
        estimate_msg = PoseWithCovarianceStamped()
        estimate_msg.header.stamp = self.get_clock().now().to_msg()
        estimate_msg.header.frame_id = "map"
        estimate_msg.pose.pose.position.x = estimated_state[0]
        estimate_msg.pose.pose.position.y = estimated_state[1]
        estimate_msg.pose.pose.orientation.z = np.sin(estimated_state[2] / 2.0)
        estimate_msg.pose.pose.orientation.w = np.cos(estimated_state[2] / 2.0)

        # Rellenar la matriz de covarianza
        for i in range(6):
            for j in range(6):
                estimate_msg.pose.covariance[i * 6 + j] = estimated_covariance[i, j]

        # Publicar el mensaje
        self.publisher.publish(estimate_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

