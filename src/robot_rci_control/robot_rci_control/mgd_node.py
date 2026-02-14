#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np


class MGDNode(Node):
    """Modèle Géométrique Direct"""

    def __init__(self):
        super().__init__('mgd_node')

        # Paramètres géométriques du robot
        self.a = 1.85
        self.b = 0.35

        # Subscriber aux états des joints
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher pour la position de l'effecteur
        self.publisher = self.create_publisher(
            PointStamped,
            'end_effector_position',
            10
        )

        self.get_logger().info('MGD Node démarré')

    def mgd(self, q):
        """
        Calcul du Modèle Géométrique Direct
        Entrée: q = [q1, q2, q3, q4] (angles en radians et translation en mètres)
        Sortie: p3 = [X, Y, Z] (position cartésienne de l'effecteur)
        """
        q1 = q[0]
        q2 = q[1]
        q4 = q[3]

        xp3 = (self.a + (q4 + self.b) * np.sin(q2)) * np.sin(q1)
        yp3 = -(self.a + (q4 + self.b) * np.sin(q2)) * np.cos(q1)
        zp3 = (q4 + self.b) * np.cos(q2) + 0.2

        return np.array([xp3, yp3, zp3])

    def joint_state_callback(self, msg):
        """Callback pour les états des joints"""
        if len(msg.position) >= 4:
            q = msg.position[:4]

            # Calcul MGD
            position = self.mgd(q)

            # Publication de la position
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'world'
            point_msg.point.x = float(position[0])
            point_msg.point.y = float(position[1])
            point_msg.point.z = float(position[2])

            self.publisher.publish(point_msg)

            self.get_logger().debug(
                f'MGD: q=[{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}] -> '
                f'p=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MGDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
