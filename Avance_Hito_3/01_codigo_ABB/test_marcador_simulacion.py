"""Regresión del manipulador usando mensajes/servidor simulados, sin ROS físico."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch
import unittest
import numpy as np


class Pose:
    def __init__(self):
        self.position = SimpleNamespace(x=0., y=0., z=0.)
        self.orientation = SimpleNamespace(x=0., y=0., z=0., w=1.)


def load_marker():
    modules = {}
    for name in ('geometry_msgs', 'geometry_msgs.msg', 'interactive_markers',
                 'interactive_markers.interactive_marker_server', 'interactive_markers.menu_handler',
                 'visualization_msgs', 'visualization_msgs.msg'):
        modules[name] = ModuleType(name)
    modules['geometry_msgs.msg'].Pose = Pose
    modules['geometry_msgs.msg'].PointStamped = object
    modules['interactive_markers.interactive_marker_server'].InteractiveMarkerServer = object
    modules['interactive_markers.menu_handler'].MenuHandler = object
    for name in ('InteractiveMarker', 'InteractiveMarkerControl', 'Marker'):
        setattr(modules['visualization_msgs.msg'], name, object)
    modules['visualization_msgs.msg'].InteractiveMarkerFeedback = SimpleNamespace(MOUSE_DOWN=1, POSE_UPDATE=2, MOUSE_UP=3)
    spec = importlib.util.spec_from_file_location('_marker_regression', Path(__file__).with_name('objetivo_rviz.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class MarkerPreviewTest(unittest.TestCase):
    def setUp(self):
        self.module = load_marker()
        self.marker = self.module.TipTarget.__new__(self.module.TipTarget)
        m = self.marker
        m.enabled = True
        m.last_xyz, m.last_rotation = (0., 0., .5), np.eye(3)
        m.dragging = m.drag_changed = False
        m.visible_controls = ['sphere', 'axis']
        m.marker = SimpleNamespace(controls=list(m.visible_controls), pose=Pose())
        m.server, m.menu = Mock(), Mock()
        m.callback, m.orientation_callback = Mock(), Mock()

    def test_preview_hides_axes_ignores_feedback_and_restores_at_new_tip(self):
        m = self.marker
        m.set_enabled(False)
        m.server.erase.assert_called_once_with(self.module.MARKER_NAME)
        m.server.insert.assert_not_called()
        m.set_enabled(False)
        m.server.erase.assert_called_once()
        feedback = SimpleNamespace(header=SimpleNamespace(frame_id='world'), event_type=2, pose=Pose())
        m.feedback(feedback)
        m.orientation_callback.assert_not_called()
        rotation = self.module.rotation_rpy([10., 20., 30.])
        m.set_tip((.1, .2, .6), rotation)
        m.set_tip((.2, .3, .7), rotation)
        m.server.setPose.assert_not_called()
        m.server.insert.assert_not_called()
        m.set_enabled(True)
        self.assertEqual(m.marker.controls, m.visible_controls)
        m.server.insert.assert_called_once()
        pose = m.server.insert.call_args.args[0].pose
        self.assertEqual(pose.position.z, .7)
        np.testing.assert_allclose(self.module.quaternion_rotation([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]), rotation)

    def test_same_pose_does_not_republish_or_overwrite_orientation(self):
        m = self.marker
        rotation = self.module.rotation_rpy([15., 25., 35.])
        m.set_tip((.1, .2, .3), rotation)
        m.server.setPose.reset_mock()
        m.set_tip((.1, .2, .3), rotation)
        m.server.setPose.assert_not_called()
        np.testing.assert_allclose(m.last_rotation, rotation)
