import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.kf = KalmanFilter(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep
        # TODO: Run predict() and update() of KalmanFilter
        # TODO: Publish estimated state
        pass

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
