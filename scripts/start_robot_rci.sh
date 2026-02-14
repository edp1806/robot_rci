#!/bin/bash

# Couleurs pour les messages
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}=================================================="
echo -e "🤖 DÉMARRAGE ROBOT RCI"
echo -e "==================================================${NC}\n"

# Source ROS2 Jazzy (CORRIGÉ)
echo -e "${GREEN}📦 Chargement de ROS2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Source workspace
echo -e "${GREEN}📦 Chargement du workspace...${NC}"
source ~/Bureau/robot_rci_ws/install/setup.bash

# Lancement
echo -e "${GREEN}🚀 Lancement de RViz + GUI...${NC}\n"
ros2 launch robot_rci_description robot_complete.launch.py

echo -e "\n${RED}👋 Arrêt du robot RCI${NC}"
