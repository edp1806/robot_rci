#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


class TrajectoryNode(Node):
    """Génération de trajectoire circulaire"""

    def __init__(self):
        super().__init__('trajectory_node')

        # Paramètres du cercle
        self.declare_parameter('center_x', -1.70)
        self.declare_parameter('center_y', -1.10)
        self.declare_parameter('center_z', 0.55)
        self.declare_parameter('radius', 0.20)
        self.declare_parameter('num_points', 60)
        self.declare_parameter('frequency', 10.0)

        # Récupération des paramètres
        self.xc = self.get_parameter('center_x').value
        self.yc = self.get_parameter('center_y').value
        self.zc = self.get_parameter('center_z').value
        self.R = self.get_parameter('radius').value
        num_points = self.get_parameter('num_points').value
        freq = self.get_parameter('frequency').value

        # Génération de la trajectoire circulaire
        theta = np.linspace(0, 2*np.pi, num_points)
        self.X = self.xc + self.R * np.cos(theta)
        self.Y = self.yc + self.R * np.sin(theta)
        self.Z = self.zc * np.ones_like(theta)

        self.current_index = 0

        # Publisher pour la position cible
        self.position_publisher = self.create_publisher(
            PointStamped,
            'target_position',
            10
        )

        # Publisher pour la visualisation du cercle
        self.marker_publisher = self.create_publisher(
            Marker,
            'trajectory_marker',
            10
        )

        # Timer pour publier les points de la trajectoire
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

        # Publier le marqueur du cercle une fois
        self.publish_circle_marker()

        self.get_logger().info(
            f'Trajectory Node démarré - Cercle: centre=({self.xc}, {self.yc}, {self.zc}), '
            f'rayon={self.R}, points={num_points}'
        )

    def publish_circle_marker(self):
        """Publie un marqueur pour visualiser le cercle dans RViz"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Style du marqueur
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Points du cercle
        for i in range(len(self.X)):
            p = Point()
            p.x = float(self.X[i])
            p.y = float(self.Y[i])
            p.z = float(self.Z[i])
            marker.points.append(p)

        # Fermer le cercle
        p = Point()
        p.x = float(self.X[0])
        p.y = float(self.Y[0])
        p.z = float(self.Z[0])
        marker.points.append(p)

        self.marker_publisher.publish(marker)

    def timer_callback(self):
        """Callback du timer pour publier le point suivant"""
        # Position actuelle
        x = self.X[self.current_index]
        y = self.Y[self.current_index]
        z = self.Z[self.current_index]

        # Publication
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'world'
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = float(z)

        self.position_publisher.publish(point_msg)

        # Passer au point suivant
        self.current_index = (self.current_index + 1) % len(self.X)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
