# 🤖 Robot RCI - Contrôle et Visualisation

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.12-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Status](https://img.shields.io/badge/Status-Active-success)

Interface graphique complète de type industriel pour le contrôle d'un robot SCARA 4-DOF avec modèles géométriques direct (MGD) et inverse (MGI), visualisation RViz 3D, et trajectoires automatiques.

![Interface Screenshot](docs/screenshot.png)

---

## 📋 Table des matières

- [Caractéristiques](#-caractéristiques)
- [Prérequis](#-prérequis)
- [Installation](#-installation)
- [Utilisation](#-utilisation)
- [Architecture du robot](#-architecture-du-robot)
- [Modèles cinématiques](#-modèles-cinématiques)
- [Trajectoires disponibles](#-trajectoires-disponibles)
- [Structure du projet](#-structure-du-projet)
- [Dépannage](#-dépannage)
- [Développement futur](#-développement-futur)
- [Auteur](#-auteur)

---

## ✨ Caractéristiques

### 🎮 Interface de contrôle complète

- **Contrôle articulaire (MGD)** : Contrôle direct des 4 articulations (q₁, q₂, q₃, q₄)
- **Contrôle cartésien (MGI)** : Contrôle de la position de l'effecteur (X, Y, Z)
- **Mise à jour temps réel** : Synchronisation bidirectionnelle entre espaces articulaire et cartésien
- **Design industriel moderne** : Interface dark mode professionnelle avec affichage digital

### 📊 Visualisation avancée

- **RViz intégré** : Visualisation 3D du robot avec modèle URDF complet
- **Espace de travail** : Calcul et affichage du volume atteignable (24,000 points)
- **Trajectoires en temps réel** : Visualisation des mouvements avec marqueurs colorés
- **Publication continue** : Maintien de position via timer ROS2 (10 Hz)

### 🔄 Trajectoires automatiques

- ⭕ **Cercle** : Trajectoire circulaire horizontale
- ⬛ **Carré** : Trajectoire carrée avec coins précis
- 〰️ **Vague** : Onde sinusoïdale 2D
- ∞ **Lemniscate** : Symbole infini (courbe en 8)

### ✅ Validation

- **MGD ↔ MGI** : Vérification de cohérence avec précision < 1 μm
- **Limites articulaires** : Respect automatique des butées mécaniques
- **Espace atteignable** : Visualisation des zones accessibles

---

## 🔧 Prérequis

### Système d'exploitation

- **Ubuntu 24.04 LTS** (recommandé)
- **ROS2 Jazzy Jalisco**

### Dépendances

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    python3-pip \
    python3-tk
```

## 📥 Installation
# 1. Cloner le repository

```bash
cd ~/Bureau
git clone https://github.com/edp1806/robot_rci.git robot_rci_ws
cd robot_rci_ws
```

# 2. Compiler le workspace

```bash
colcon build
source install/setup.bash
```

# 3. Vérifier l'installation

```bash
ros2 pkg list | grep robot_rci
```

Vous devriez voir :

text
robot_rci_description
robot_rci_gui

***🚀 Utilisation
**Lancement complet

# Terminal 1 : Lancer RViz et le modèle URDF

```bash
cd ~/Bureau/robot_rci_ws
source install/setup.bash
ros2 launch robot_rci_description display.launch.py
```
# Terminal 2 : Lancer l'interface de contrôle

```bash
cd ~/Bureau/robot_rci_ws
source install/setup.bash
ros2 run robot_rci_gui control_panel
```
# Configuration RViz

    Fixed Frame : world

    Ajouter les éléments :

        RobotModel : Add → RobotModel

        Workspace : Add → By topic → /workspace_marker → Marker

        Trajectoire : Add → By topic → /trajectory_marker → Marker

### Utilisation de l'interface
## Contrôle articulaire (MGD)

    Déplacez les curseurs J1, J2, J3, J4

    La position cartésienne (X, Y, Z) se met à jour automatiquement

## Contrôle cartésien (MGI)

    Déplacez les curseurs X, Y, Z

    Les angles articulaires se calculent automatiquement

## Affichage de l'espace de travail

    Cliquez sur "WORKSPACE DISPLAY"

    Attendre 3-5 secondes (calcul de 24,000 points)

    L'espace apparaît en bleu dans RViz

## Exécution de trajectoires

    Sélectionnez un type : Circle, Square, Wave ou Lemniscate

    Cliquez sur "START TRAJECTORY"

    Le robot suit la trajectoire en boucle

    Cliquez sur "STOP TRAJECTORY" pour arrêter

## Validation des modèles

    Cliquez sur "VALIDATE MGD ↔ MGI"

    Une fenêtre affiche l'erreur de précision (doit être < 1 μm)

### 🦾 Architecture du robot
Paramètres géométriques
Paramètre	Valeur	Description
a	1.85 m	Longueur du bras principal
b	0.35 m	Décalage de l'outil
Base height	0.50 m	Hauteur de la base
Max extension	0.15 m	Extension maximale prismatique (J4)
Limites articulaires
Joint	Type	Limites	Unité
J1	Revolute	-110° à +110°	deg
J2	Revolute	0° à +80°	deg
J3	Revolute	0° à +360°	deg
J4	Prismatic	0 à 15	cm
DOF (Degrés de liberté)

Configuration : 3R + 1P (3 rotations + 1 translation)

### 📐 Modèles cinématiques
## Modèle Géométrique Direct (MGD)

Calcul de la position cartésienne P(X, Y, Z) à partir des angles articulaires q = [q₁, q₂, q₃, q₄] :

text
L_total = b + q₄ + 0.15

X = (a + L_total × sin(q₂)) × sin(q₁)
Y = -(a + L_total × sin(q₂)) × cos(q₁)
Z = L_total × cos(q₂) + 0.5

## Modèle Géométrique Inverse (MGI)

Calcul des angles articulaires q à partir de la position P(X, Y, Z) :

text
q₁ = arctan2(X, -Y)
q₂ = arctan2(√(X² + Y²) - a, Z - 0.5)
q₃ = 0
q₄ = √((√(X² + Y²) - a)² + (Z - 0.5)²) - b - 0.15

Avec respect des limites articulaires via clipping.
## Validation

    Précision MGD ↔ MGI : < 1 μm (10⁻⁶ m)

    Méthode : Test de cohérence avec erreur euclidienne

### 🎯 Trajectoires disponibles
Trajectoire	Description	Paramètres clés	Couleur
⭕ Cercle	Cercle horizontal	Rayon: 0.20 m, 60 points	🟢 Vert
⬛ Carré	Carré horizontal	Côté: 0.30 m, 4×15 points	🔵 Bleu
〰️ Vague	Onde sinusoïdale 2D	Amplitude: 0.15 m, 2 périodes	🟠 Orange
∞ Lemniscate	Symbole infini	Échelle: 0.15 m, 80 points	🟡 Jaune
Centre des trajectoires

Toutes les trajectoires sont centrées sur :

    X : -1.70 m

    Y : -1.10 m

    Z : 1.00 m

Fréquence de suivi

    Taux de rafraîchissement : 20 Hz (50 ms par point)

    Republication marqueurs : 1 Hz

## 📁 Structure du projet
```
text
robot_rci_ws/
├── src/
│   ├── robot_rci_description/          # Description URDF du robot
│   │   ├── urdf/
│   │   │   └── robot_rci.urdf.xacro   # Modèle 3D complet du robot
│   │   ├── meshes/                     # Modèles 3D (STL/DAE)
│   │   │   ├── base.stl
│   │   │   ├── link1.stl
│   │   │   ├── link2.stl
│   │   │   └── tool.stl
│   │   ├── launch/
│   │   │   └── display.launch.py      # Lancement RViz + URDF
│   │   ├── rviz/
│   │   │   └── robot_config.rviz      # Configuration RViz sauvegardée
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── robot_rci_gui/                  # Interface de contrôle
│       ├── robot_rci_gui/
│       │   ├── __init__.py
│       │   └── control_panel.py       # Interface graphique principale
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
│
├── build/                               # Fichiers de compilation (gitignore)
├── install/                             # Fichiers installés (gitignore)
├── log/                                 # Logs ROS2 (gitignore)
├── .gitignore
└── README.md
```

### 🔍 Dépannage
Le robot revient à sa position initiale dans RViz

➡️ Normal ! Le timer ROS2 publie continuellement les joint_states pour maintenir la position. C'est un comportement attendu.
Erreur No module named 'tkinter'

'''bash
sudo apt install python3-tk
'''

RViz ne montre pas le robot

    Vérifiez que robot_state_publisher tourne :

    '''bash
    ros2 node list
    '''

    Ajoutez manuellement : Add → RobotModel

    Changez Fixed Frame en world

L'espace de travail n'apparaît pas

    Attendez le calcul complet (24,000 points = ~3-5 secondes)

    Dans RViz : Add → By topic → /workspace_marker → Marker

    Vérifiez les logs dans le terminal du control_panel

La trajectoire ne s'affiche pas dans RViz

Dans RViz : Add → By topic → /trajectory_marker → Marker
Erreur de compilation colcon build

'''bash
# Nettoyer le workspace
rm -rf build/ install/ log/

# Sourcer ROS2
source /opt/ros/jazzy/setup.bash

# Recompiler
colcon build
'''
Le GUI ne se lance pas

'''bash
# Vérifier l'installation du package
ros2 pkg list | grep robot_rci_gui

# Recompiler si nécessaire
cd ~/Bureau/robot_rci_ws
colcon build --packages-select robot_rci_gui
source install/setup.bash
'''
### 🚧 Développement futur
## Fonctionnalités prévues

    Planification de trajectoire avec évitement d'obstacles

    Support de la cinématique différentielle (jacobienne)

    Interface web avec ROS2 Bridge

    Contrôle par joystick/gamepad

    Enregistrement et replay de trajectoires personnalisées

    Intégration MoveIt2 pour planification avancée

    Simulation Gazebo avec physique réaliste

    Mode "teach pendant" (apprentissage par démonstration)

    Export des trajectoires en format CSV/JSON

## Améliorations techniques

    Optimisation du calcul de l'espace de travail (GPU)

    Interpolation de trajectoires (splines cubiques)

    Gestion de singularités cinématiques

    Contrôle en effort (force feedback)

## 🤝 Contribuer

Les contributions sont les bienvenues ! Pour contribuer :

    Forkez le projet

    Créez une branche (git checkout -b feature/AmazingFeature)

    Committez vos changements (git commit -m 'Add some AmazingFeature')

    Pushez vers la branche (git push origin feature/AmazingFeature)

    Ouvrez une Pull Request

## 📄 Licence

Ce projet est sous licence MIT - voir le fichier LICENSE pour plus de détails.

## 👤 Auteur

Étudiant Polytech Lille

    Spécialisation : Systèmes embarqués, énergie et industrie 4.0

    Formation : Ingénieur en Systèmes embarqués et génie
    Électrique

    GitHub : @edp1806

    Projet : Robot RCI - Station de contrôle industrielle

🙏 Remerciements

    Polytech Lille - Département SE

    Communauté ROS2 - Documentation et outils exceptionnels

    Open Robotics - Développement de ROS2 et RViz

    Équipe pédagogique - Encadrement et support technique

📊 Statistiques du projet

    Lignes de code Python : ~850

    Espace de travail : 24,000 points calculés

    Précision cinématique : < 1 μm

    Taux de rafraîchissement GUI : 10 Hz

    Fréquence trajectoires : 20 Hz

<div align="center">

⭐ Si ce projet vous aide, n'hésitez pas à lui donner une étoile ! ⭐
Made with ❤️  at Polytech Lille
</div> ```
