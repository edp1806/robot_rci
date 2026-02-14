#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


class WorkspacePublisher(Node):
    """Publie l'espace de travail du robot pour visualisation dans RViz"""

    def __init__(self):
        super().__init__('workspace_publisher')

        # Paramètres géométriques
        self.a = 1.85
        self.b = 0.35

        # Publisher pour le marqueur d'espace de travail
        self.publisher = self.create_publisher(
            Marker,
            'workspace_marker',
            10
        )

        # Timer pour publier périodiquement
        self.timer = self.create_timer(2.0, self.publish_workspace)

        # Calculer l'espace de travail
        self.workspace_points = self.calculate_workspace()

        self.get_logger().info(
            f'Workspace Publisher démarré - {len(self.workspace_points)} points calculés'
        )

    def mgd(self, q):
        """Modèle Géométrique Direct"""
        q1, q2, q4 = q
        xp3 = (self.a + (q4 + self.b) * np.sin(q2)) * np.sin(q1)
        yp3 = -(self.a + (q4 + self.b) * np.sin(q2)) * np.cos(q1)
        zp3 = (q4 + self.b) * np.cos(q2) + 0.2
        return np.array([xp3, yp3, zp3])

    def calculate_workspace(self):
        """Calcule l'espace de travail atteignable"""
        q1_values = np.linspace(np.deg2rad(-110), np.deg2rad(110), 40)
        q2_values = np.linspace(np.deg2rad(0), np.deg2rad(80), 30)
        q4_values = np.linspace(0, 0.15, 20)

        points = []
        for q1 in q1_values:
            for q2 in q2_values:
                for q4 in q4_values:
                    p = self.mgd([q1, q2, q4])
                    points.append(p)

        return np.array(points)

    def publish_workspace(self):
        """Publie le marqueur de l'espace de travail"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Style
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.3

        # Ajouter tous les points
        for p in self.workspace_points:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = float(p[2])
            marker.points.append(point)

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
