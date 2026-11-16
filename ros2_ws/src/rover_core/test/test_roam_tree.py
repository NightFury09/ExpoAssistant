"""The roam recovery tree must never spin or reverse the rover. No ROS needed.

    python3 -m pytest ros2_ws/src/rover_core/test/test_roam_tree.py

Roam goals carry roam_recovery.xml; every other goal uses booth_recovery.xml. The
booth tree spins and backs up on purpose. Roam must not: its only sensing is one
lidar plane at 27.5 cm, so a spin's collision check cannot see low or very close
objects, and on 2026-11-12 a roam recovery spin swept the chassis into one.
"""
import os
import xml.etree.ElementTree as ET

import pytest

TREES = os.path.join(os.path.dirname(__file__), '..', '..', 'my_robot_bringup', 'behavior_trees')
ROAM = os.path.join(TREES, 'roam_recovery.xml')
BOOTH = os.path.join(TREES, 'booth_recovery.xml')
MOTION = {'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop'}   # recoveries that move the rover


def tags(path):
    return [e.tag for e in ET.parse(path).getroot().iter()]


def test_roam_tree_is_well_formed():
    root = ET.parse(ROAM).getroot()
    assert root.tag == 'root'
    main = root.get('main_tree_to_execute')
    assert main and any(t.get('ID') == main for t in root.iter('BehaviorTree'))


@pytest.mark.parametrize('node', sorted(MOTION))
def test_roam_tree_has_no_motion_recoveries(node):
    assert node not in tags(ROAM)


def test_booth_tree_still_has_them():
    # The point of a separate tree: GO and SET GOAL keep the booth behaviour.
    assert {'Spin', 'BackUp'} <= set(tags(BOOTH))


def test_roam_tree_uses_only_nodes_the_booth_tree_already_loads():
    # booth_recovery.xml is known to load in this Nav2 install, so staying inside
    # its vocabulary means no new plugin has to be registered.
    assert set(tags(ROAM)) <= set(tags(BOOTH)) | {'root'}


def test_roam_tree_gives_up_quickly():
    # Few retries: a stuck roaming rover should abort and let roam pick another
    # spot, not thrash.
    retries = [int(e.get('number_of_retries', 0)) for e in ET.parse(ROAM).getroot().iter('RecoveryNode')
               if e.get('name') == 'NavigateRecovery']
    assert retries and max(retries) <= 3
