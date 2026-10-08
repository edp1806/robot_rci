"""
Tests des modèles géométriques et des trajectoires du panneau de contrôle.

Ils appellent les vraies méthodes de RobotControlGUI (mgd, mgi, is_reachable,
generate_*) sans démarrer ROS ni l'interface Tkinter. Les chiffres du README
(précision MGD <-> MGI, bornes de l'espace de travail, trajectoires
atteignables) viennent de ces tests.

Lancement (depuis la racine du workspace, après colcon build) :
    source install/setup.bash
    python3 -m pytest src/robot_rci_gui/test -v
"""

import numpy as np
import pytest

from robot_rci_gui.control_panel import RobotControlGUI

TOL = 1e-9  # 1 nm


@pytest.fixture
def robot():
    # __new__ : on évite Node.__init__ (pas besoin de rclpy.init ni d'affichage)
    gui = RobotControlGUI.__new__(RobotControlGUI)
    gui.a = 1.85
    gui.b = 0.35
    return gui


def workspace_grid():
    """Même grille que calculate_workspace() : 40 x 30 x 20 = 24 000 configurations"""
    for q1 in np.linspace(np.deg2rad(-110), np.deg2rad(110), 40):
        for q2 in np.linspace(np.deg2rad(0), np.deg2rad(80), 30):
            for q4 in np.linspace(0, 0.15, 20):
                yield np.array([q1, q2, 0.0, q4])


def test_validation_button_point(robot):
    """Configuration testée par le bouton VALIDATE MGD <-> MGI"""
    q = np.array([np.deg2rad(30), np.deg2rad(40), 0.0, 0.12])
    p = robot.mgd(q)
    assert np.linalg.norm(p - robot.mgd(robot.mgi(p))) < TOL


def test_round_trip_whole_workspace(robot):
    """MGD -> MGI -> MGD sur les 24 000 configurations de l'espace de travail"""
    errors = [np.linalg.norm(robot.mgd(q) - robot.mgd(robot.mgi(robot.mgd(q))))
              for q in workspace_grid()]
    assert len(errors) == 24000
    assert max(errors) < TOL


def test_workspace_bounds_match_gui_label(robot):
    """Les bornes affichées dans le GUI correspondent à la grille calculée"""
    pts = np.array([robot.mgd(q) for q in workspace_grid()])
    assert pts[:, 0].min() == pytest.approx(-2.49, abs=0.005)
    assert pts[:, 0].max() == pytest.approx(2.49, abs=0.005)
    assert pts[:, 1].min() == pytest.approx(-2.49, abs=0.005)
    assert pts[:, 1].max() == pytest.approx(0.85, abs=0.005)
    assert pts[:, 2].min() == pytest.approx(0.59, abs=0.005)
    assert pts[:, 2].max() == pytest.approx(1.15, abs=0.005)


@pytest.mark.parametrize('name', ['circle', 'square', 'wave', 'lemniscate'])
def test_trajectory_fully_reachable(robot, name):
    """Chaque point est atteignable et l'effecteur passe exactement dessus"""
    X, Y, Z = getattr(robot, f'generate_{name}')()
    for target in zip(X, Y, Z):
        assert robot.is_reachable(target), f'{name}: point {target} hors d\'atteinte'
        reached = robot.mgd(robot.mgi(target))
        assert np.linalg.norm(reached - np.array(target)) < TOL


def test_is_reachable_rejects_points_outside(robot):
    assert not robot.is_reachable((0.0, -1.80, 1.0))   # trop près : q2 < 0
    assert not robot.is_reachable((0.0, -2.40, 1.0))   # trop loin : q4 > 15 cm
    assert not robot.is_reachable((0.0, 2.00, 1.0))    # derrière : |q1| > 110°
    assert robot.is_reachable((0.0, -2.05, 1.0))
