"""Gedeelde pytest-fixtures voor de my_bot unit tests.

patrol_node.py is een los uitvoerbaar script (het pakket heeft geen __init__.py /
setup.py), dus we voegen de map met de node-scripts toe aan sys.path zodat we het
als module kunnen importeren. De ROS-onderdelen worden in de `node`-fixture gemockt,
zodat we puur de state machine testen — zonder netwerk, Nav2 of hardware.
"""
import os
import sys
from unittest.mock import MagicMock

import pytest
import rclpy

_NODES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'my_bot'))
if _NODES_DIR not in sys.path:
    sys.path.insert(0, _NODES_DIR)


@pytest.fixture(scope='session', autouse=True)
def _ros_context():
    """Eén rclpy-context voor de hele testsessie."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    """Een verse PatrolNode per test met gemockte ROS-I/O.

    De echte ActionClient en CancelGoal-client worden vervangen door mocks, zodat
    callbacks direct aangeroepen kunnen worden zonder draaiende Nav2-server.
    """
    from patrol_node import PatrolNode

    n = PatrolNode()
    n._action_client = MagicMock()
    n._action_client.wait_for_server.return_value = True
    n._action_client.send_goal_async.return_value = MagicMock()
    n._cancel_client = MagicMock()
    # Voortgang-I/O mocken zodat tests deterministisch zijn en niet het echte
    # voortgangsbestand op schijf lezen/schrijven.
    n._load_progress = MagicMock(return_value=0)
    n._save_progress = MagicMock()
    n._clear_progress = MagicMock()
    yield n
    n.destroy_node()
