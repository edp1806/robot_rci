# 🤖 Robot RCI - Guide de Compilation et d'Exécution

Ce guide vous explique comment compiler et exécuter le projet Robot RCI sous ROS2 Jazzy.

---

## 📋 Prérequis

### Système d'exploitation
- Ubuntu 24.04 LTS (Noble Numbat)

### Logiciels requis
```bash
# Mise à jour du système
sudo apt update && sudo apt upgrade -y

# Installation de ROS2 Jazzy (si pas déjà installé)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Installation des dépendances du projet
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    python3-tk \
    python3-numpy

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🏗️ Compilation du Workspace

### 1. Navigation vers le workspace
```bash
cd ~/robot_rci_ws
```

### 2. Compilation
```bash
colcon build --symlink-install
```

**Options de compilation :**
- `--symlink-install` : Crée des liens symboliques au lieu de copier les fichiers (utile pour le développement)
- `--packages-select <nom_package>` : Compile uniquement un package spécifique

**Exemple :** Compiler uniquement `robot_rci_control`
```bash
colcon build --symlink-install --packages-select robot_rci_control
```

### 3. Source du workspace
```bash
source install/setup.bash
```

**Astuce :** Ajoutez ceci à votre `~/.bashrc` pour sourcer automatiquement :
```bash
echo "source ~/robot_rci_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🚀 Utilisation du Robot

### Mode 1 : Visualisation RViz uniquement (sans physique)

Idéal pour tester la cinématique sans simulation complète.
```bash
ros2 launch robot_rci_description display.launch.py
```

**Ce qui se lance :**
- RViz2 avec le modèle du robot
- `robot_state_publisher` : Publie les transformations TF
- `joint_state_publisher_gui` : Interface pour bouger les joints manuellement

---

### Mode 2 : Simulation Gazebo complète

Simulation physique réaliste du robot.
```bash
ros2 launch robot_rci_description gazebo.launch.py
```

**Ce qui se lance :**
- Gazebo avec le robot spawné
- `robot_state_publisher` : Publie les transformations TF

---

### Mode 3 : Contrôle avec nœuds MGD/MGI

#### Terminal 1 : Lancer les nœuds de contrôle
```bash
ros2 launch robot_rci_control control.launch.py
```

**Ce qui se lance :**
- `mgd_node` : Calcule la position cartésienne à partir des joints
- `mgi_node` : Calcule les joints à partir d'une position cartésienne cible
- `workspace_publisher` : Publie l'espace de travail (nuage de points bleu)

#### Terminal 2 : Interface graphique
```bash
ros2 run robot_rci_gui control_panel
```

**Fonctionnalités :**
- Curseurs articulaires (q1, q2, q3, q4)
- Curseurs cartésiens (X, Y, Z)
- Affichage position actuelle
- Boutons validation et trajectoire

---

## 🧪 Tests et Validations

### Tâche 3 : Validation croisée MGD ↔ MGI

Vérifie que MGD(MGI(p)) = p et MGI(MGD(q)) = q
```bash
ros2 run robot_rci_control validation_node
```

**Résultat attendu :**
```
=== VALIDATION MGD <-> MGI ===
Configuration initiale q: [30.0, 40.0] (deg), q3=0.000 rad, q4=0.120 m
MGD(q) = p3: [X, Y, Z] m
MGI(p3) = qd: [q1, q2] (deg), q3 rad, q4 m
Erreur position: [1e-15, 1e-15, 1e-15] m
✓ VALIDATION RÉUSSIE
```

---

### Tâche 4 : Trajectoire circulaire

Lance le robot sur une trajectoire circulaire dans l'espace de travail.
```bash
ros2 launch robot_rci_control trajectory.launch.py
```

**Paramètres modifiables :**
```bash
ros2 launch robot_rci_control trajectory.launch.py \
    center_x:=-1.70 \
    center_y:=-1.10 \
    center_z:=0.55 \
    radius:=0.20 \
    num_points:=60 \
    frequency:=10.0
```

---

## 📊 Visualisation dans RViz

### Ajouter l'espace de travail
1. Ouvrir RViz
2. Cliquer sur **Add** → **By topic**
3. Sélectionner `/workspace_marker` → **Marker**

### Ajouter la trajectoire
1. Cliquer sur **Add** → **By topic**
2. Sélectionner `/trajectory_marker` → **Marker**

### Ajouter la position de l'effecteur
1. Cliquer sur **Add** → **By topic**
2. Sélectionner `/end_effector_position` → **PointStamped**

---

## 🛠️ Commandes utiles

### Lister les nœuds actifs
```bash
ros2 node list
```

### Lister les topics
```bash
ros2 topic list
```

### Voir les messages d'un topic
```bash
ros2 topic echo /end_effector_position
ros2 topic echo /joint_states
```

### Publier manuellement une position cible
```bash
ros2 topic pub /target_position geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'world'}, point: {x: -1.5, y: -1.0, z: 0.6}}"
```

### Enregistrer les données (rosbag)
```bash
ros2 bag record -a  # Enregistre tous les topics
```

### Rejouer les données
```bash
ros2 bag play <nom_du_bag>
```

---

## 🐛 Dépannage

### Erreur : "package not found"
```bash
# Re-sourcer le workspace
source ~/robot_rci_ws/install/setup.bash

# Vérifier que le package est bien compilé
colcon list
```

### Erreur : "No module named 'tkinter'"
```bash
sudo apt install python3-tk
```

### Gazebo ne se lance pas
```bash
# Installer/réinstaller Gazebo
sudo apt install ros-jazzy-gazebo-ros-pkgs
```

### RViz ne montre pas le robot
1. Vérifier que `Fixed Frame` = `world`
2. Ajouter un display **RobotModel**
3. Vérifier le topic `/robot_description`

---

## 📁 Structure des Topics ROS2

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | État actuel des joints |
| `/joint_commands` | `sensor_msgs/JointState` | Commandes pour les joints |
| `/target_position` | `geometry_msgs/PointStamped` | Position cartésienne cible |
| `/end_effector_position` | `geometry_msgs/PointStamped` | Position actuelle de l'effecteur |
| `/workspace_marker` | `visualization_msgs/Marker` | Espace de travail (nuage bleu) |
| `/trajectory_marker` | `visualization_msgs/Marker` | Trajectoire circulaire (vert) |
| `/robot_description` | `std_msgs/String` | Description URDF du robot |

---

## 🎯 Scénarios d'utilisation typiques

### Scénario 1 : Test rapide de la visualisation
```bash
# Terminal 1
ros2 launch robot_rci_description display.launch.py
# Bouger les curseurs dans joint_state_publisher_gui
```

### Scénario 2 : Contrôle interactif complet
```bash
# Terminal 1
ros2 launch robot_rci_description display.launch.py

# Terminal 2
ros2 launch robot_rci_control control.launch.py

# Terminal 3
ros2 run robot_rci_gui control_panel
```

### Scénario 3 : Simulation Gazebo + Trajectoire
```bash
# Terminal 1
ros2 launch robot_rci_description gazebo.launch.py

# Terminal 2
ros2 launch robot_rci_control trajectory.launch.py
```

---

## 📚 Ressources supplémentaires

- [Documentation ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Tutoriels URDF](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo + ROS2](https://gazebosim.org/docs/latest/ros2_integration)
- [RViz2 User Guide](https://github.com/ros2/rviz/blob/jazzy/README.md)

---

## ✅ Checklist de démarrage

- [ ] ROS2 Jazzy installé
- [ ] Dépendances installées (`gazebo-ros`, `rviz2`, etc.)
- [ ] Workspace compilé (`colcon build`)
- [ ] Workspace sourcé (`source install/setup.bash`)
- [ ] Test RViz OK (`ros2 launch robot_rci_description display.launch.py`)
- [ ] Test nœuds OK (`ros2 launch robot_rci_control control.launch.py`)
- [ ] Interface GUI OK (`ros2 run robot_rci_gui control_panel`)

---

**Bon développement ! 🚀**
