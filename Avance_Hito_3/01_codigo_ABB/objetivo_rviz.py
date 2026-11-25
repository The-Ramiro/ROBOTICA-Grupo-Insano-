"""Entrada cartesiana de RViz. Publica únicamente un destino de vista previa."""
import math
import numpy as np

from calculos_robotica import quaternion_rotation, rotation_quaternion

from geometry_msgs.msg import PointStamped, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from calculos_robotica import rotation_rpy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

NAMESPACE = '/imt342/objetivo'
POINT_TOPIC = '/imt342/punto'
MARKER_NAME = 'brida_destino'


def position(point):
    xyz = (point.x, point.y, point.z)
    return xyz if all(map(math.isfinite, xyz)) else None


class TipTarget:
    def __init__(self, node, callback, xyz):
        self.callback = callback
        self.orientation_callback = None
        self.last_rotation = np.eye(3)
        self.dragging = False
        self.enabled = True
        self.drag_changed = False
        self.translation_step = .001
        self.rotation_step = 1.
        self.mode = 'move'
        self.server = InteractiveMarkerServer(node, NAMESPACE)
        self.points = node.create_subscription(PointStamped, POINT_TOPIC, self.clicked, 5)
        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = MARKER_NAME
        marker.scale = .15
        marker.pose = self.pose(xyz)

        handle = InteractiveMarkerControl()
        handle.name = 'arrastrar'
        handle.description = 'Arrastrar en el plano de la vista'
        handle.orientation.w = 1.
        handle.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        handle.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        handle.always_visible = True
        ball = Marker(type=Marker.SPHERE)
        ball.pose.orientation.w = 1.
        ball.scale.x = ball.scale.y = ball.scale.z = .036
        ball.color.g, ball.color.b, ball.color.a = .86, .93, 1.
        handle.markers.append(ball)
        marker.controls.append(handle)

        root = math.sqrt(.5)
        for axis, quaternion in (('X', (0., 0., 0., 1.)), ('Y', (0., 0., root, root)),
                                 ('Z', (0., -root, 0., root))):
            control = InteractiveMarkerControl()
            control.name = axis
            control.description = f'Desplazar en {axis}'
            control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quaternion
            control.orientation_mode = InteractiveMarkerControl.FIXED
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

        self.server.insert(marker, feedback_callback=self.feedback)
        self.server.applyChanges()
        self.last_xyz = tuple(xyz)
        self.marker = marker
        self.controls = list(marker.controls)

    @staticmethod
    def pose(xyz, rotation=None):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        quaternion = (0., 0., 0., 1.) if rotation is None else rotation_quaternion(rotation)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quaternion)
        return pose

    def enable_rotation(self, callback, rotation):
        self.orientation_callback = callback
        root = math.sqrt(.5)
        for axis, quaternion in (('X', (0., 0., 0., 1.)), ('Y', (0., 0., root, root)),
                                 ('Z', (0., -root, 0., root))):
            control = InteractiveMarkerControl()
            control.name = 'girar_' + axis
            control.description = 'Girar alrededor de ' + axis
            control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quaternion
            control.orientation_mode = InteractiveMarkerControl.INHERIT
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            self.marker.controls.append(control)
        self.last_rotation = np.array(rotation, copy=True)
        self.marker.pose = self.pose(self.last_xyz, rotation)
        self.controls = list(self.marker.controls)
        self.controls[0].interaction_mode = InteractiveMarkerControl.MENU
        self.controls[0].description = 'Clic derecho para ajustes precisos'
        self.controls[0].markers[0].scale.x = .024
        self.controls[0].markers[0].scale.y = .024
        self.controls[0].markers[0].scale.z = .024
        self.configure()

    def configure(self, mode='move', translation_step_mm=1., rotation_step_deg=1.):
        if mode not in ('move', 'rotate', 'both'):
            raise ValueError('Modo de manipulador desconocido.')
        if not all(math.isfinite(v) and v > 0 for v in (translation_step_mm, rotation_step_deg)):
            raise ValueError('Los pasos de ajuste deben ser positivos.')
        self.mode = mode
        self.translation_step, self.rotation_step = translation_step_mm/1000., rotation_step_deg
        self.marker.controls = [self.controls[0]]
        for control in self.controls[1:]:
            rotating = control.name.startswith('girar_')
            if mode == 'both' or (rotating and mode == 'rotate') or (not rotating and mode == 'move'):
                self.marker.controls.append(control)
        self.visible_controls = list(self.marker.controls)
        if not self.enabled:
            self.marker.controls = self.visible_controls[:1]
        self.marker.pose = self.pose(self.last_xyz, self.last_rotation)
        self.menu = MenuHandler()
        for axis, name in enumerate('XYZ'):
            move = self.menu.insert('Mover '+name)
            turn = self.menu.insert('Girar eje local '+name)
            for direction in (-1, 1):
                self.menu.insert(f'{direction*translation_step_mm:+g} mm', parent=move,
                                 callback=lambda _, a=axis, d=direction: self.nudge(a, d, False))
                self.menu.insert(f'{direction*rotation_step_deg:+g}°', parent=turn,
                                 callback=lambda _, a=axis, d=direction: self.nudge(a, d, True))
        if self.enabled:
            self.server.insert(self.marker, feedback_callback=self.feedback)
            self.menu.apply(self.server, MARKER_NAME)
        else:
            self.server.erase(MARKER_NAME)
        self.server.applyChanges()

    def set_enabled(self, enabled):
        if self.enabled == enabled:
            return
        self.enabled = enabled
        self.dragging = self.drag_changed = False
        # Borrar el marcador completo de RViz evita controles residuales/parpadeos.
        self.marker.controls = list(self.visible_controls)
        self.marker.pose = self.pose(self.last_xyz, self.last_rotation)
        if enabled:
            self.server.insert(self.marker, feedback_callback=self.feedback)
            self.menu.apply(self.server, MARKER_NAME)
        else:
            self.server.erase(MARKER_NAME)
        self.server.applyChanges()

    def nudge(self, axis, direction, rotating):
        if not self.enabled:
            return
        self.dragging = False
        self.drag_changed = False
        xyz, rotation = np.asarray(self.last_xyz).copy(), self.last_rotation.copy()
        if rotating:
            degrees = np.zeros(3)
            degrees[axis] = direction*self.rotation_step
            rotation = rotation @ rotation_rpy(degrees)
        else:
            xyz[axis] += direction*self.translation_step
        if self.orientation_callback is not None:
            self.orientation_callback(tuple(xyz), rotation, True)

    def set_tip(self, xyz, rotation=None):
        xyz = tuple(xyz)
        rotation = self.last_rotation if rotation is None else np.asarray(rotation)
        changed_rotation = not np.allclose(rotation, self.last_rotation, atol=1e-10, rtol=0.)
        if not self.dragging and (xyz != self.last_xyz or changed_rotation):
            if self.enabled:
                self.server.setPose(MARKER_NAME, self.pose(xyz, rotation))
                self.server.applyChanges()
            self.marker.pose = self.pose(xyz, rotation)
            self.last_xyz = xyz
            self.last_rotation = rotation.copy()

    def feedback(self, feedback):
        if not self.enabled:
            return
        if feedback.header.frame_id != 'world':
            return
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True
            self.drag_changed = False
            return
        if feedback.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            return
        final = feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP
        self.dragging = not final
        self.server.applyChanges()
        xyz = position(feedback.pose.position)
        orientation = feedback.pose.orientation
        try:
            rotation = quaternion_rotation((orientation.x, orientation.y, orientation.z, orientation.w))
        except ValueError:
            return
        rotated = self.orientation_callback is not None and not np.allclose(
            rotation, self.last_rotation, atol=1e-10, rtol=0.)
        if xyz is not None and (xyz != self.last_xyz or rotated or (final and self.drag_changed)):
            self.drag_changed = not final
            self.last_xyz = xyz
            self.last_rotation = rotation
            if self.orientation_callback is None:
                self.callback(xyz, final)
            else:
                self.orientation_callback(xyz, rotation, final)

    def clicked(self, message):
        if not self.enabled:
            return
        if message.header.frame_id != 'world':
            return
        xyz = position(message.point)
        if xyz is not None:
            self.callback(xyz, True)

    def close(self):
        self.server.shutdown()
