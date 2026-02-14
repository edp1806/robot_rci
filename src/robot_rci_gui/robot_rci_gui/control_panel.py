#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import time
import datetime


class RobotControlGUI(Node):
    """Interface graphique complète pour contrôler le robot RCI"""

    def __init__(self):
        super().__init__('robot_control_gui')

        # Paramètres géométriques du robot
        self.a = 1.85   # Longueur du bras
        self.b = 0.35   # Décalage de l'outil

        # Publishers ROS2
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.workspace_pub = self.create_publisher(Marker, 'workspace_marker', 10)
        self.trajectory_pub = self.create_publisher(Marker, 'trajectory_marker', 10)

        # État du robot
        self.current_q = np.array([np.deg2rad(30), np.deg2rad(40), 0.0, 0.0])
        self.trajectory_running = False
        self.workspace_visible = False
        self.workspace_points = None

        # Flag pour éviter les boucles infinies
        self.updating = False

        # Timer pour publication continue (empêche le retour à la position initiale)
        self.publish_timer = self.create_timer(0.1, self.continuous_publish)

        self.get_logger().info('🤖 Robot Control GUI - Initialisation...')

    # ========================================
    # MODÈLES CINÉMATIQUES
    # ========================================

    def mgd(self, q):
        """Modèle Géométrique Direct"""
        q1, q2, q3, q4 = q
        L_total = self.b + q4 + 0.15
        X = (self.a + L_total * np.sin(q2)) * np.sin(q1)
        Y = -(self.a + L_total * np.sin(q2)) * np.cos(q1)
        Z = L_total * np.cos(q2) + 0.5
        return np.array([X, Y, Z])

    def mgi(self, position):
        """Modèle Géométrique Inverse"""
        X, Y, Z = position
        q1 = np.arctan2(X, -Y)
        q2 = np.arctan2(np.sqrt(X**2 + Y**2) - self.a, Z - 0.5)
        q3 = 0.0
        distance_totale = np.sqrt((np.sqrt(X**2 + Y**2) - self.a)**2 + (Z - 0.5)**2)
        q4 = distance_totale - self.b - 0.15
        q1 = np.clip(q1, -110*np.pi/180, 110*np.pi/180)
        q2 = np.clip(q2, 0, 80*np.pi/180)
        q4 = np.clip(q4, 0, 0.15)
        return np.array([q1, q2, q3, q4])

    # ========================================
    # PUBLICATION CONTINUE
    # ========================================

    def continuous_publish(self):
        """Publie continuellement la position actuelle"""
        self.publish_joints(self.current_q)

    def publish_joints(self, q):
        """Publie les positions articulaires"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        if isinstance(q, np.ndarray):
            msg.position = q.tolist()
        else:
            msg.position = list(q)
        self.joint_pub.publish(msg)

    # ========================================
    # ESPACE DE TRAVAIL
    # ========================================

    def calculate_workspace(self):
        """Calcule l'espace de travail atteignable du robot"""
        self.get_logger().info('⏳ Calcul de l\'espace de travail...')

        q1_values = np.linspace(np.deg2rad(-110), np.deg2rad(110), 40)
        q2_values = np.linspace(np.deg2rad(0), np.deg2rad(80), 30)
        q4_values = np.linspace(0, 0.15, 20)

        points = []
        for q1 in q1_values:
            for q2 in q2_values:
                for q4 in q4_values:
                    p = self.mgd([q1, q2, 0, q4])
                    points.append(p)

        self.workspace_points = np.array(points)

        x_min, x_max = self.workspace_points[:, 0].min(), self.workspace_points[:, 0].max()
        y_min, y_max = self.workspace_points[:, 1].min(), self.workspace_points[:, 1].max()
        z_min, z_max = self.workspace_points[:, 2].min(), self.workspace_points[:, 2].max()

        self.get_logger().info(f'✅ Espace de travail calculé: {len(points)} points')
        self.get_logger().info(f'   X: [{x_min:.3f}, {x_max:.3f}] m')
        self.get_logger().info(f'   Y: [{y_min:.3f}, {y_max:.3f}] m')
        self.get_logger().info(f'   Z: [{z_min:.3f}, {z_max:.3f}] m')

        for _ in range(5):
            self.publish_workspace()
            time.sleep(0.2)

    def publish_workspace(self):
        """Publie l'espace de travail pour visualisation dans RViz"""
        if not self.workspace_visible or self.workspace_points is None:
            return

        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8

        marker.lifetime.sec = 0

        for p in self.workspace_points:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = float(p[2])
            marker.points.append(point)

        self.workspace_pub.publish(marker)
        self.get_logger().info(f'📊 Espace de travail publié: {len(marker.points)} points')

    def toggle_workspace(self):
        """Active/désactive l'affichage de l'espace de travail"""
        self.workspace_visible = not self.workspace_visible

        if self.workspace_visible:
            self.workspace_btn.config(text="HIDE WORKSPACE", bg="#9e5a2b")
            self.get_logger().info('🌐 Activation espace de travail...')

            if self.workspace_points is None:
                threading.Thread(target=self.calculate_workspace, daemon=True).start()
            else:
                for _ in range(5):
                    self.publish_workspace()
                    time.sleep(0.2)
        else:
            self.workspace_btn.config(text="WORKSPACE DISPLAY", bg="#2b5a9e")

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'workspace'
            marker.id = 0
            marker.action = Marker.DELETE
            self.workspace_pub.publish(marker)

            self.get_logger().info('🌐 Espace de travail masqué')

    # ========================================
    # VALIDATION MGD ↔ MGI
    # ========================================

    def validate_models(self):
        """Valide la cohérence entre MGD et MGI"""
        q_test = np.array([np.deg2rad(30), np.deg2rad(40), 0.0, 0.12])
        p_forward = self.mgd(q_test)
        q_inverse = self.mgi(p_forward)
        p_check = self.mgd(q_inverse)
        error_p = p_forward - p_check
        norm_error = np.linalg.norm(error_p)

        result = "═══════════════════════════════════\n"
        result += "   VALIDATION MGD ↔ MGI\n"
        result += "═══════════════════════════════════\n\n"
        result += f"Erreur position: ||p - p'|| = {norm_error:.6e} m\n\n"

        if norm_error < 1e-6:
            result += "✅ VALIDATION RÉUSSIE!\n"
            result += "   Précision < 1 μm"
        else:
            result += f"⚠️ Erreur: {norm_error*1000:.3f} mm"

        messagebox.showinfo("Validation MGD ↔ MGI", result)
        self.get_logger().info(f'Validation: erreur = {norm_error:.6e} m')

    # ========================================
    # TRAJECTOIRES
    # ========================================

    def toggle_trajectory(self):
        """Démarre ou arrête la trajectoire"""
        if not self.trajectory_running:
            self.start_trajectory()
        else:
            self.stop_trajectory()

    def start_trajectory(self):
        """Démarre la trajectoire sélectionnée"""
        self.trajectory_running = True
        self.traj_btn.config(text="STOP TRAJECTORY", bg="#2b9e2b")
        self.status_indicator.itemconfig(self.status_circle, fill="#ffaa00", outline="#ffaa00")

        # Récupérer le type de trajectoire sélectionné
        self.trajectory_type = self.traj_type_var.get()

        threading.Thread(target=self.run_trajectory, daemon=True).start()
        self.get_logger().info(f'🔵 Trajectoire {self.trajectory_type} démarrée')

    def stop_trajectory(self):
        """Arrête la trajectoire"""
        self.trajectory_running = False
        self.traj_btn.config(text="START TRAJECTORY", bg="#9e2b2b")
        self.status_indicator.itemconfig(self.status_circle, fill="#00ff00", outline="#00ff00")

        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectory'
        marker.id = 0
        marker.action = Marker.DELETE
        self.trajectory_pub.publish(marker)

        self.get_logger().info('⏹ Trajectoire arrêtée')

    def run_trajectory(self):
        """Exécute la trajectoire sélectionnée"""
        # Générer les points selon le type
        if self.trajectory_type == "circle":
            X, Y, Z = self.generate_circle()
            color = (0.0, 1.0, 0.0)  # Vert
        elif self.trajectory_type == "square":
            X, Y, Z = self.generate_square()
            color = (0.0, 0.5, 1.0)  # Bleu
        elif self.trajectory_type == "wave":
            X, Y, Z = self.generate_wave()
            color = (1.0, 0.5, 0.0)  # Orange
        elif self.trajectory_type == "lemniscate":
            X, Y, Z = self.generate_lemniscate()
            color = (1.0, 1.0, 0.0)  # Jaune
        else:
            X, Y, Z = self.generate_circle()
            color = (0.0, 1.0, 0.0)

        # Publier la trajectoire plusieurs fois
        for _ in range(5):
            self.publish_trajectory_marker(X, Y, Z, color)
            time.sleep(0.1)

        # Thread pour republier périodiquement
        def republish():
            while self.trajectory_running:
                self.publish_trajectory_marker(X, Y, Z, color)
                time.sleep(1.0)

        threading.Thread(target=republish, daemon=True).start()

        # Suivre la trajectoire
        i = 0
        while self.trajectory_running:
            target_position = [X[i], Y[i], Z[i]]

            # Vérifier si la position est atteignable
            q = self.mgi(target_position)

            # Mise à jour du GUI
            self.root.after(0, lambda q=q, p=target_position: self.update_from_trajectory(q, p))

            i = (i + 1) % len(X)
            time.sleep(0.05)


    # ========================================
    # GÉNÉRATEURS DE TRAJECTOIRES
    # ========================================

    def generate_circle(self):
        """Génère un cercle horizontal"""
        center_x = -1.70
        center_y = -1.10
        center_z = 1.00
        radius = 0.20
        num_points = 60

        theta = np.linspace(0, 2*np.pi, num_points)
        X = center_x + radius * np.cos(theta)
        Y = center_y + radius * np.sin(theta)
        Z = center_z * np.ones_like(theta)

        return X, Y, Z

    def generate_square(self):
        """Génère un carré horizontal"""
        center_x = -1.70
        center_y = -1.10
        center_z = 1.00
        side = 0.30
        points_per_side = 15

        X, Y, Z = [], [], []

        # Côté 1: bas (gauche → droite)
        for i in range(points_per_side):
            t = i / (points_per_side - 1)
            X.append(center_x - side/2 + side * t)
            Y.append(center_y - side/2)
            Z.append(center_z)

        # Côté 2: droite (bas → haut)
        for i in range(points_per_side):
            t = i / (points_per_side - 1)
            X.append(center_x + side/2)
            Y.append(center_y - side/2 + side * t)
            Z.append(center_z)

        # Côté 3: haut (droite → gauche)
        for i in range(points_per_side):
            t = i / (points_per_side - 1)
            X.append(center_x + side/2 - side * t)
            Y.append(center_y + side/2)
            Z.append(center_z)

        # Côté 4: gauche (haut → bas)
        for i in range(points_per_side):
            t = i / (points_per_side - 1)
            X.append(center_x - side/2)
            Y.append(center_y + side/2 - side * t)
            Z.append(center_z)

        return np.array(X), np.array(Y), np.array(Z)

    def generate_wave(self):
        """Génère une trajectoire en vague sinusoïdale"""
        x_start = -1.90
        x_end = -1.50
        y_center = -1.10
        z_center = 1.00
        amplitude = 0.15
        wavelength = 2
        num_points = 60

        X = np.linspace(x_start, x_end, num_points)
        t = np.linspace(0, wavelength * 2*np.pi, num_points)
        Y = y_center + amplitude * np.sin(t)
        Z = z_center * np.ones_like(X)

        return X, Y, Z

    def generate_lemniscate(self):
        """Génère une lemniscate (symbole infini ∞)"""
        center_x = -1.70
        center_y = -1.10
        center_z = 1.00
        scale = 0.15
        num_points = 80

        t = np.linspace(0, 2*np.pi, num_points)

        # Équation paramétrique de la lemniscate
        X = center_x + scale * np.cos(t) / (1 + np.sin(t)**2)
        Y = center_y + scale * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
        Z = center_z * np.ones_like(t)

        return X, Y, Z

    def publish_trajectory_marker(self, X, Y, Z, color):
        """Publie la trajectoire pour RViz"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        marker.lifetime.sec = 0

        for i in range(len(X)):
            p = Point()
            p.x = float(X[i])
            p.y = float(Y[i])
            p.z = float(Z[i])
            marker.points.append(p)

        # Fermer la trajectoire pour cercle/carré/lemniscate
        if self.trajectory_type in ["circle", "square", "lemniscate"]:
            p = Point()
            p.x = float(X[0])
            p.y = float(Y[0])
            p.z = float(Z[0])
            marker.points.append(p)

        self.trajectory_pub.publish(marker)
        self.get_logger().info(f'🟢 Trajectoire {self.trajectory_type} publiée')

    # ========================================
    # CONTRÔLE DU ROBOT
    # ========================================

    def on_joint_change(self, _):
        """Callback: Curseurs articulaires modifiés (MGD)"""
        if self.updating:
            return

        self.updating = True

        q = np.array([
            np.deg2rad(self.q1_var.get()),
            np.deg2rad(self.q2_var.get()),
            np.deg2rad(self.q3_var.get()),
            self.q4_var.get() / 100.0
        ])

        self.current_q = q
        position = self.mgd(q)

        self.x_var.set(round(position[0], 3))
        self.y_var.set(round(position[1], 3))
        self.z_var.set(round(position[2], 3))

        self.update_position_display(position)
        self.updating = False

    def on_cartesian_change(self, _):
        """Callback: Curseurs cartésiens modifiés (MGI)"""
        if self.updating:
            return

        self.updating = True

        position = np.array([
            self.x_var.get(),
            self.y_var.get(),
            self.z_var.get()
        ])

        q = self.mgi(position)
        self.current_q = q

        self.q1_var.set(round(np.rad2deg(q[0]), 2))
        self.q2_var.set(round(np.rad2deg(q[1]), 2))
        self.q3_var.set(round(np.rad2deg(q[2]), 2))
        self.q4_var.set(round(q[3] * 100, 2))

        self.update_position_display(position)
        self.updating = False

    def update_from_trajectory(self, q, position):
        """Met à jour le GUI depuis la trajectoire"""
        if self.updating:
            return

        self.updating = True
        self.current_q = q

        self.q1_var.set(round(np.rad2deg(q[0]), 2))
        self.q2_var.set(round(np.rad2deg(q[1]), 2))
        self.q3_var.set(round(np.rad2deg(q[2]), 2))
        self.q4_var.set(round(q[3] * 100, 2))

        self.x_var.set(round(position[0], 3))
        self.y_var.set(round(position[1], 3))
        self.z_var.set(round(position[2], 3))

        self.update_position_display(position)
        self.updating = False

    def update_position_display(self, position):
        """Met à jour l'affichage de la position"""
        text = f"X: {position[0]:+.3f}\nY: {position[1]:+.3f}\nZ: {position[2]:+.3f}"
        self.pos_label.config(text=text)

    # ========================================
    # INTERFACE GRAPHIQUE PROFESSIONNELLE
    # ========================================

    def create_gui(self):
        """Crée l'interface graphique professionnelle de type industriel"""
        self.get_logger().info('Création de l\'interface graphique...')

        self.root = tk.Tk()
        self.root.title("Robot RCI Control Station")
        self.root.geometry("1200x800")
        self.root.configure(bg="#1a1a1a")

        # Style professionnel
        style = ttk.Style()
        style.theme_use('clam')

        # ========================================
        # BARRE DE TITRE PROFESSIONNELLE
        # ========================================
        header = tk.Frame(self.root, bg="#2b2b2b", height=80, relief='raised', bd=2)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)

        # Logo/Titre
        title_frame = tk.Frame(header, bg="#2b2b2b")
        title_frame.pack(side="left", padx=20, pady=10)

        tk.Label(title_frame, text="ROBOT RCI",
                font=("Arial", 24, "bold"), bg="#2b2b2b", fg="#00ff00").pack(anchor="w")
        tk.Label(title_frame, text="Industrial Control Station v2.0",
                font=("Consolas", 9), bg="#2b2b2b", fg="#888888").pack(anchor="w")

        # Statut système
        status_frame = tk.Frame(header, bg="#2b2b2b")
        status_frame.pack(side="right", padx=20, pady=10)

        self.status_indicator = tk.Canvas(status_frame, width=20, height=20, bg="#2b2b2b",
                                          highlightthickness=0)
        self.status_indicator.pack(side="left", padx=5)
        self.status_circle = self.status_indicator.create_oval(2, 2, 18, 18, fill="#00ff00", outline="#00ff00")

        tk.Label(status_frame, text="SYSTEM READY", font=("Consolas", 11, "bold"),
                bg="#2b2b2b", fg="#00ff00").pack(side="left", padx=5)

        # ========================================
        # CONTENEUR PRINCIPAL AVEC 2 COLONNES
        # ========================================
        main_container = tk.Frame(self.root, bg="#1a1a1a")
        main_container.pack(fill="both", expand=True, padx=10, pady=10)

        # COLONNE GAUCHE - CONTRÔLES
        left_panel = tk.Frame(main_container, bg="#1a1a1a")
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # COLONNE DROITE - STATUS ET ACTIONS
        right_panel = tk.Frame(main_container, bg="#1a1a1a", width=350)
        right_panel.pack(side="right", fill="both", padx=(5, 0))
        right_panel.pack_propagate(False)

        # ========================================
        # CONTRÔLE ARTICULAIRE (GAUCHE)
        # ========================================
        frame_joint = tk.LabelFrame(left_panel, text="JOINT CONTROL (MGD)",
                                     bg="#2b2b2b", fg="#00ff00",
                                     font=("Consolas", 10, "bold"),
                                     bd=2, relief='solid', padx=15, pady=15)
        frame_joint.pack(fill="x", pady=(0, 10))

        self.q1_var = tk.DoubleVar(value=30.0)
        self.q2_var = tk.DoubleVar(value=40.0)
        self.q3_var = tk.DoubleVar(value=0.0)
        self.q4_var = tk.DoubleVar(value=0.0)

        joints_config = [
            ("J1", self.q1_var, -110, 110, "°", "#ff4444"),
            ("J2", self.q2_var, 0, 80, "°", "#44ff44"),
            ("J3", self.q3_var, 0, 360, "°", "#4444ff"),
            ("J4", self.q4_var, 0, 15, "cm", "#ffaa00")
        ]

        for i, (label, var, min_val, max_val, unit, color) in enumerate(joints_config):
            row_frame = tk.Frame(frame_joint, bg="#2b2b2b")
            row_frame.pack(fill="x", pady=8)

            # Label joint avec couleur
            label_frame = tk.Frame(row_frame, bg=color, width=45, height=30)
            label_frame.pack(side="left", padx=(0, 10))
            label_frame.pack_propagate(False)
            tk.Label(label_frame, text=label, font=("Consolas", 12, "bold"),
                    bg=color, fg="#000000").pack(expand=True)

            # Slider
            slider = tk.Scale(row_frame, from_=min_val, to=max_val, variable=var,
                             orient="horizontal", length=450, resolution=0.1,
                             bg="#3a3a3a", fg="#ffffff", troughcolor="#1a1a1a",
                             activebackground=color, highlightthickness=0,
                             font=("Consolas", 9), command=self.on_joint_change)
            slider.pack(side="left", padx=5)

            # Valeur digitale
            value_frame = tk.Frame(row_frame, bg="#000000", width=85, height=30,
                                  relief="sunken", bd=2)
            value_frame.pack(side="left", padx=(10, 0))
            value_frame.pack_propagate(False)
            value_label = tk.Label(value_frame, textvariable=var,
                                  font=("Consolas", 12, "bold"),
                                  bg="#000000", fg="#00ff00")
            value_label.pack(expand=True)

            # Unité
            tk.Label(row_frame, text=unit, font=("Consolas", 10),
                    bg="#2b2b2b", fg="#888888").pack(side="left", padx=(5, 0))

        # ========================================
        # CONTRÔLE CARTÉSIEN (GAUCHE)
        # ========================================
        frame_cart = tk.LabelFrame(left_panel, text="CARTESIAN CONTROL (MGI)",
                                    bg="#2b2b2b", fg="#00ff00",
                                    font=("Consolas", 10, "bold"),
                                    bd=2, relief='solid', padx=15, pady=15)
        frame_cart.pack(fill="x", pady=(0, 10))

        p3_init = self.mgd(self.current_q)
        self.x_var = tk.DoubleVar(value=round(p3_init[0], 3))
        self.y_var = tk.DoubleVar(value=round(p3_init[1], 3))
        self.z_var = tk.DoubleVar(value=round(p3_init[2], 3))

        cartesian_config = [
            ("X", self.x_var, -3, 3, "#ff0000"),
            ("Y", self.y_var, -3, 3, "#00ff00"),
            ("Z", self.z_var, 0, 1.5, "#0066ff")
        ]

        for i, (label, var, min_val, max_val, color) in enumerate(cartesian_config):
            row_frame = tk.Frame(frame_cart, bg="#2b2b2b")
            row_frame.pack(fill="x", pady=8)

            # Label axe
            label_frame = tk.Frame(row_frame, bg=color, width=45, height=30)
            label_frame.pack(side="left", padx=(0, 10))
            label_frame.pack_propagate(False)
            tk.Label(label_frame, text=label, font=("Consolas", 12, "bold"),
                    bg=color, fg="#ffffff").pack(expand=True)

            # Slider
            slider = tk.Scale(row_frame, from_=min_val, to=max_val, variable=var,
                             orient="horizontal", length=450, resolution=0.001,
                             bg="#3a3a3a", fg="#ffffff", troughcolor="#1a1a1a",
                             activebackground=color, highlightthickness=0,
                             font=("Consolas", 9), command=self.on_cartesian_change)
            slider.pack(side="left", padx=5)

            # Valeur digitale
            value_frame = tk.Frame(row_frame, bg="#000000", width=85, height=30,
                                  relief="sunken", bd=2)
            value_frame.pack(side="left", padx=(10, 0))
            value_frame.pack_propagate(False)
            value_label = tk.Label(value_frame, textvariable=var,
                                  font=("Consolas", 12, "bold"),
                                  bg="#000000", fg="#00ff00")
            value_label.pack(expand=True)

            # Unité
            tk.Label(row_frame, text="m", font=("Consolas", 10),
                    bg="#2b2b2b", fg="#888888").pack(side="left", padx=(5, 0))

        # ========================================
        # POSITION STATUS (DROITE)
        # ========================================
        frame_status = tk.LabelFrame(right_panel, text="END EFFECTOR POSITION",
                                     bg="#1a1a1a", fg="#ffffff",
                                     font=("Consolas", 10, "bold"),
                                     bd=2, relief='solid', padx=15, pady=15)
        frame_status.pack(fill="x", pady=(0, 10))

        # Display digital
        display_frame = tk.Frame(frame_status, bg="#000000", relief="sunken", bd=3)
        display_frame.pack(fill="x", pady=10)

        self.pos_label = tk.Label(display_frame,
                                  text=f"X: {p3_init[0]:+.3f}\nY: {p3_init[1]:+.3f}\nZ: {p3_init[2]:+.3f}",
                                  font=("Consolas", 14, "bold"), bg="#000000", fg="#00ff00",
                                  justify="left", padx=20, pady=15)
        self.pos_label.pack()

        # Indicateurs de limites
        limits_frame = tk.Frame(frame_status, bg="#1a1a1a")
        limits_frame.pack(fill="x", pady=(10, 0))

        tk.Label(limits_frame, text="WORKSPACE LIMITS", font=("Consolas", 8, "bold"),
                bg="#1a1a1a", fg="#888888").pack()
        tk.Label(limits_frame, text="X: [-2.34, 2.34] | Y: [-2.34, 0.80] | Z: [0.71, 1.15]",
                font=("Consolas", 7), bg="#1a1a1a", fg="#666666").pack()

        # ========================================
        # PANNEAU DE CONTRÔLE (DROITE)
        # ========================================
        frame_actions = tk.LabelFrame(right_panel, text="OPERATION PANEL",
                                    bg="#1a1a1a", fg="#ffffff",
                                    font=("Consolas", 10, "bold"),
                                    bd=2, relief='solid', padx=15, pady=15)
        frame_actions.pack(fill="x", pady=(0, 10))

        # Bouton workspace
        self.workspace_btn = tk.Button(frame_actions, text="WORKSPACE DISPLAY",
                                        command=self.toggle_workspace,
                                        font=("Consolas", 10, "bold"),
                                        bg="#2b5a9e", fg="#ffffff",
                                        activebackground="#3d7dd6",
                                        activeforeground="#ffffff",
                                        relief="raised", bd=3,
                                        padx=20, pady=12, cursor="hand2")
        self.workspace_btn.pack(fill="x", pady=(0, 10))

        # Bouton validation
        validate_btn = tk.Button(frame_actions, text="VALIDATE MGD ↔ MGI",
                                command=self.validate_models,
                                font=("Consolas", 10, "bold"),
                                bg="#2b9e5a", fg="#ffffff",
                                activebackground="#3dd67d",
                                activeforeground="#ffffff",
                                relief="raised", bd=3,
                                padx=20, pady=12, cursor="hand2")
        validate_btn.pack(fill="x", pady=(0, 10))

        # NOUVEAU : Sélection du type de trajectoire
        traj_select_frame = tk.Frame(frame_actions, bg="#1a1a1a")
        traj_select_frame.pack(fill="x", pady=(0, 10))

        tk.Label(traj_select_frame, text="TRAJECTORY TYPE:",
                font=("Consolas", 8, "bold"),
                bg="#1a1a1a", fg="#888888").pack(anchor="w", pady=(0, 5))

        self.traj_type_var = tk.StringVar(value="circle")

        trajectories = [
            ("⭕ Circle", "circle", "#00ff00"),
            ("⬛ Square", "square", "#0088ff"),
            ("〰️ Wave", "wave", "#ff8800"),
            ("∞ Lemniscate", "lemniscate", "#ffff00")
        ]

        for i, (text, value, color) in enumerate(trajectories):
            rb = tk.Radiobutton(traj_select_frame, text=text, variable=self.traj_type_var,
                            value=value, font=("Consolas", 9),
                            bg="#1a1a1a", fg=color, selectcolor="#2b2b2b",
                            activebackground="#1a1a1a", activeforeground=color,
                            highlightthickness=0, bd=0)
            rb.pack(anchor="w", pady=2)

        # Bouton trajectoire
        self.traj_btn = tk.Button(frame_actions, text="START TRAJECTORY",
                                command=self.toggle_trajectory,
                                font=("Consolas", 10, "bold"),
                                bg="#9e2b2b", fg="#ffffff",
                                activebackground="#d63d3d",
                                activeforeground="#ffffff",
                                relief="raised", bd=3,
                                padx=20, pady=12, cursor="hand2")
        self.traj_btn.pack(fill="x")

        # ========================================
        # INFORMATIONS SYSTÈME (DROITE BAS)
        # ========================================
        frame_info = tk.LabelFrame(right_panel, text="SYSTEM INFO",
                                   bg="#1a1a1a", fg="#ffffff",
                                   font=("Consolas", 10, "bold"),
                                   bd=2, relief='solid', padx=15, pady=15)
        frame_info.pack(fill="both", expand=True)

        info_text = tk.Text(frame_info, height=10, font=("Consolas", 8),
                           bg="#1a1a1a", fg="#00ff00", relief="sunken", bd=2,
                           highlightthickness=0, wrap="word")
        info_text.pack(fill="both", expand=True)

        info_content = """ROBOT CONFIGURATION
━━━━━━━━━━━━━━━━━━━━
DOF: 4 (3R + 1P)
Workspace: 24,000 points
Base height: 0.50 m
Arm length: 1.85 m
Tool offset: 0.35 m
Max extension: 0.15 m

JOINT LIMITS
━━━━━━━━━━━━━━━━━━━━
J1: -110° to +110°
J2: 0° to +80°
J3: 0° to +360°
J4: 0 to 15 cm

STATUS: OPERATIONAL
"""
        info_text.insert("1.0", info_content)
        info_text.config(state="disabled")



        # ========================================
        # BARRE DE STATUS (BAS)
        # ========================================
        footer = tk.Frame(self.root, bg="#2b2b2b", height=30, relief='raised', bd=1)
        footer.pack(fill="x", side="bottom")
        footer.pack_propagate(False)

        tk.Label(footer, text="ROS2 Jazzy | URDF Model Active | RViz Connected",
                font=("Consolas", 8), bg="#2b2b2b", fg="#888888").pack(side="left", padx=10)

        time_label = tk.Label(footer, text=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                             font=("Consolas", 8), bg="#2b2b2b", fg="#888888")
        time_label.pack(side="right", padx=10)

        def update_time():
            time_label.config(text=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            self.root.after(1000, update_time)

        update_time()

        self.get_logger().info('✅ Interface graphique créée!')

    def spin(self):
        """Boucle principale avec intégration ROS2"""
        def ros_update():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.after(10, ros_update)

        ros_update()
        self.root.mainloop()


def main(args=None):
    print("\n" + "="*50)
    print("🚀 ROBOT RCI CONTROL STATION")
    print("="*50 + "\n")

    rclpy.init(args=args)

    try:
        gui = RobotControlGUI()
        gui.create_gui()

        print("✅ Industrial Control Interface Ready")
        print("\n📋 Features:")
        print("  • Joint Control (MGD)")
        print("  • Cartesian Control (MGI)")
        print("  • Workspace Display (24,000 points)")
        print("  • MGD ↔ MGI Validation")
        print("  • Circular Trajectory")
        print("\n⚠️  RViz Setup:")
        print("  Add → By topic → workspace_marker → Marker")
        print("  Add → By topic → trajectory_marker → Marker")
        print("\n" + "="*50 + "\n")

        gui.spin()

    except KeyboardInterrupt:
        print("\n⏹ System Shutdown")
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("\n👋 Goodbye!\n")


if __name__ == '__main__':
    main()
