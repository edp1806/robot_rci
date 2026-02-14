# 🤖 Robot RCI - Contrôle et Visualisation

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> Interface graphique complète pour le contrôle d'un robot SCARA avec modèles géométriques direct (MGD) et inverse (MGI), visualisation RViz, et trajectoires automatiques.

![Robot RCI Interface](docs/screenshot.png)

---

## 📋 Table des matières

- [Caractéristiques](#-caractéristiques)
- [Prérequis](#-prérequis)
- [Installation](#-installation)
- [Utilisation](#-utilisation)
- [Architecture](#-architecture)
- [Modèles cinématiques](#-modèles-cinématiques)
- [Contribuer](#-contribuer)
- [Licence](#-licence)

---

## ✨ Caractéristiques

### 🎮 Interface de contrôle complète
- **Contrôle articulaire (MGD)** : Contrôle direct des 4 articulations (q1, q2, q3, q4)
- **Contrôle cartésien (MGI)** : Contrôle de la position de l'effecteur (X, Y, Z)
- **Mise à jour temps réel** : Synchronisation bidirectionnelle entre espaces articulaire et cartésien

### 📊 Visualisation avancée
- **RViz intégré** : Visualisation 3D du robot et de son environnement
- **Espace de travail** : Affichage du volume atteignable (24,000 points)
- **Trajectoires** : Visualisation en temps réel des mouvements

### 🔄 Trajectoires automatiques
- **Trajectoire circulaire** : Suivi automatique d'un cercle dans l'espace de travail
- **Validation MGD ↔ MGI** : Vérification de la cohérence des modèles cinématiques

### 🚀 Lancement simplifié
- **Icône bureau** : Lancement en un clic
- **Configuration automatique** : Tout l'environnement ROS2 est chargé automatiquement

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
