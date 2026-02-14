#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import numpy as np


class MGINode(Node):
    """Modèle Géométrique Inverse"""

    def __init__(self):
        super().__init__('mgi_node')

        # Paramètres géométriques du robot
        self.a = 1.85
        self.b = 0.35

        # Subscriber pour la position cible cartésienne
        self.subscription = self.create_subscription(
            PointStamped,
            'target_position',
            self.target_callback,
            10
        )

        # Publisher pour les commandes articulaires
        self.publisher = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        self.get_logger().info('MGI Node démarré')

    def mgi(self, position):
        """
        Calcul du Modèle Géométrique Inverse
        Entrée: position = [X, Y, Z] (position cartésienne)
        Sortie: q = [q1, q2, q3, q4] (angles et translation)
        """
        X = position[0]
        Y = position[1]
        Z = position[2]

        # Calcul de q1 (angle de rotation autour de Z)
        q1 = np.arctan2(X, -Y)

        # Calcul de q2 (angle de rotation autour de X)
        q2 = np.arctan2(np.sqrt(X**2 + Y**2) - self.a, Z - 0.2)

        # q3 fixé à 0
        q3 = 0.0

        # Calcul de q4 (translation le long de Z)
        q4 = np.sqrt((np.sqrt(X**2 + Y**2) - self.a)**2 + (Z - 0.2)**2) - self.b

        # Vérification des limites
        q1 = np.clip(q1, -110*np.pi/180, 110*np.pi/180)
        q2 = np.clip(q2, 0, 80*np.pi/180)
        q4 = np.clip(q4, 0, 0.15)

        return np.array([q1, q2, q3, q4])

    def target_callback(self, msg):
        """Callback pour la position cible"""
        position = np.array([msg.point.x, msg.point.y, msg.point.z])

        # Calcul MGI
        q = self.mgi(position)

        # Publication des commandes articulaires
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        joint_msg.position = q.tolist()

        self.publisher.publish(joint_msg)

        self.get_logger().info(
            f'MGI: p=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] -> '
            f'q=[{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MGINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
