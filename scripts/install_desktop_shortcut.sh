#!/bin/bash

echo "🔧 Installation du raccourci Robot RCI..."

# Copier sur le bureau
cp ~/Bureau/robot_rci_ws/scripts/Robot_RCI.desktop ~/Bureau/

# Permissions
chmod +x ~/Bureau/Robot_RCI.desktop

# Marquer comme trusted
gio set ~/Bureau/Robot_RCI.desktop metadata::trusted true

echo "✅ Raccourci installé sur le bureau !"
