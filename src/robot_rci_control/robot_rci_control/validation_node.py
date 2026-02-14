#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np


class ValidationNode(Node):
    """Validation croisée MGD <-> MGI (Tâche 3)"""

    def __init__(self):
        super().__init__('validation_node')

        # Paramètres géométriques
        self.a = 1.85
        self.b = 0.35

        # Timer pour exécuter la validation une fois
        self.timer = self.create_timer(1.0, self.validate)
        self.validated = False

        self.get_logger().info('Validation Node démarré')

    def mgd(self, q):
        """Modèle Géométrique Direct"""
        q1, q2, q3, q4 = q
        xp3 = (self.a + (q4 + self.b) * np.sin(q2)) * np.sin(q1)
        yp3 = -(self.a + (q4 + self.b) * np.sin(q2)) * np.cos(q1)
        zp3 = (q4 + self.b) * np.cos(q2) + 0.2
        return np.array([xp3, yp3, zp3])

    def mgi(self, position):
        """Modèle Géométrique Inverse"""
        X, Y, Z = position
        q1 = np.arctan2(X, -Y)
        q2 = np.arctan2(np.sqrt(X**2 + Y**2) - self.a, Z - 0.2)
        q3 = 0.0
        q4 = np.sqrt((np.sqrt(X**2 + Y**2) - self.a)**2 + (Z - 0.2)**2) - self.b
        return np.array([q1, q2, q3, q4])

    def validate(self):
        """Effectue la validation croisée"""
        if self.validated:
            return

        # Configuration de test
        q_test = np.array([
            np.deg2rad(30),  # q1
            np.deg2rad(40),  # q2
            0.0,             # q3
            0.12             # q4
        ])

        self.get_logger().info('=== VALIDATION MGD <-> MGI ===')
        self.get_logger().info(f'Configuration initiale q: {np.rad2deg(q_test[:2])} (deg), q3={q_test[2]:.3f} rad, q4={q_test[3]:.3f} m')

        # MGD
        p3 = self.mgd(q_test)
        self.get_logger().info(f'MGD(q) = p3: [{p3[0]:.6f}, {p3[1]:.6f}, {p3[2]:.6f}] m')

        # MGI
        qd = self.mgi(p3)
        self.get_logger().info(f'MGI(p3) = qd: [{np.rad2deg(qd[0]):.6f}, {np.rad2deg(qd[1]):.6f}] (deg), q3={qd[2]:.6f} rad, q4={qd[3]:.6f} m')

        # Vérification MGD(MGI(p3))
        p3_check = self.mgd(qd)
        self.get_logger().info(f'MGD(qd) = p3_check: [{p3_check[0]:.6f}, {p3_check[1]:.6f}, {p3_check[2]:.6f}] m')

        # Calcul des erreurs
        errQ = q_test - qd
        errXYZ = p3 - p3_check

        self.get_logger().info('=== ERREURS ===')
        self.get_logger().info(
            f'Erreur q: [{np.rad2deg(errQ[0]):.6f}°, {np.rad2deg(errQ[1]):.6f}°, '
            f'{errQ[2]:.6f} rad, {errQ[3]*100:.6f} cm]'
        )
        self.get_logger().info(
            f'Erreur position: [{errXYZ[0]:.6e}, {errXYZ[1]:.6e}, {errXYZ[2]:.6e}] m'
        )

        # Vérification de la précision
        if np.linalg.norm(errXYZ) < 1e-6:
            self.get_logger().info('✓ VALIDATION RÉUSSIE - Précision excellent!')
        else:
            self.get_logger().warn(f'⚠ Erreur de position: {np.linalg.norm(errXYZ):.6e} m')

        self.validated = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
