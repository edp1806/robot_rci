#!/usr/bin/env bash
# =====================================================================
# Robot RCI : RViz dans un serveur X virtuel (Xvnc) + panneau de contrôle
#
# Pourquoi : sous Ubuntu 26.04 (session Wayland), RViz2 passe par XWayland
# et sa vue 3D clignote (une image sur deux noire), même en rendu logiciel.
# Dans un serveur X indépendant (Xvnc), le rendu est stable. On affiche
# ce serveur dans une fenêtre vncviewer, et le panneau de contrôle Tkinter
# reste sur le bureau normal.
#
# Dépendances : sudo apt install tigervnc-standalone-server tigervnc-viewer
# Usage       : ./scripts/start_rviz_vnc.sh            (display :5)
#               VNC_DISPLAY=7 ./scripts/start_rviz_vnc.sh
# =====================================================================
set -uo pipefail

VNC_DISPLAY="${VNC_DISPLAY:-5}"
GEOMETRY="${GEOMETRY:-1280x900}"
PORT=$((5900 + VNC_DISPLAY))
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_DISPLAY="${DISPLAY:-:0}"

for cmd in Xvnc vncviewer; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "❌ $cmd introuvable : sudo apt install tigervnc-standalone-server tigervnc-viewer"
        exit 1
    fi
done

if [ -e "/tmp/.X11-unix/X${VNC_DISPLAY}" ]; then
    echo "❌ Le display :${VNC_DISPLAY} est déjà utilisé. Relance avec VNC_DISPLAY=<autre numéro>."
    exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"

PIDS=()
cleanup() {
    echo -e "\n⏹  Arrêt de RViz, du panneau et de Xvnc..."
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
}
trap cleanup EXIT INT TERM

# 1. Serveur X virtuel, accessible uniquement depuis cette machine
Xvnc ":${VNC_DISPLAY}" -geometry "${GEOMETRY}" -depth 24 \
     -SecurityTypes None -localhost -rfbport "${PORT}" \
     > "/tmp/xvnc_${VNC_DISPLAY}.log" 2>&1 &
PIDS+=($!)

for _ in $(seq 1 50); do
    [ -e "/tmp/.X11-unix/X${VNC_DISPLAY}" ] && break
    sleep 0.1
done
if [ ! -e "/tmp/.X11-unix/X${VNC_DISPLAY}" ]; then
    echo "❌ Xvnc n'a pas démarré, voir /tmp/xvnc_${VNC_DISPLAY}.log"
    exit 1
fi
echo "✅ Xvnc prêt sur :${VNC_DISPLAY} (port ${PORT}, local uniquement)"

# 2. robot_state_publisher + RViz dans le serveur virtuel
DISPLAY=":${VNC_DISPLAY}" ros2 launch robot_rci_description display.launch.py &
PIDS+=($!)

# 3. Panneau de contrôle sur le bureau normal
DISPLAY="${HOST_DISPLAY}" ros2 run robot_rci_gui control_panel &
PIDS+=($!)

sleep 3

# 4. Fenêtre d'affichage de RViz (fermer cette fenêtre arrête tout)
DISPLAY="${HOST_DISPLAY}" vncviewer "localhost::${PORT}"
