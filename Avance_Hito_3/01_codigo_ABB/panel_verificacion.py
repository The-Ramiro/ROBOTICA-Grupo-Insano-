"""Mediciones y acoplamiento de RViz para la ventana ABB."""
import math
import re
import subprocess
import time
from ctypes import CDLL, Structure, byref, c_char_p, c_int, c_long, c_uint, c_ulong, c_void_p

import numpy as np
from PyQt5 import QtCore, QtWidgets

from objetivos_preview import rotation_vector


class XWindowAttributes(Structure):
    _fields_ = [('background_pixmap', c_ulong), ('background_pixel', c_ulong),
                ('border_pixmap', c_ulong), ('border_pixel', c_ulong),
                ('bit_gravity', c_int), ('win_gravity', c_int), ('backing_store', c_int),
                ('backing_planes', c_ulong), ('backing_pixel', c_ulong), ('save_under', c_int),
                ('event_mask', c_long), ('do_not_propagate_mask', c_long), ('override_redirect', c_int),
                ('colormap', c_ulong), ('cursor', c_ulong)]


class RvizSurface(QtWidgets.QWidget):
    """Ventana hija X11. Restaura el tamaño cuando se arrastra el separador."""

    def __init__(self, identifier, parent):
        super().__init__(parent)
        self.identifier = identifier
        x11 = self.x11 = CDLL('libX11.so.6')
        x11.XOpenDisplay.argtypes = [c_char_p]
        x11.XOpenDisplay.restype = c_void_p
        for name in ('XUnmapWindow', 'XMapWindow'):
            getattr(x11, name).argtypes = [c_void_p, c_ulong]
        x11.XReparentWindow.argtypes = [c_void_p, c_ulong, c_ulong, c_int, c_int]
        x11.XChangeWindowAttributes.argtypes = [c_void_p, c_ulong, c_ulong, c_void_p]
        x11.XMoveResizeWindow.argtypes = [c_void_p, c_ulong, c_int, c_int, c_uint, c_uint]
        x11.XCloseDisplay.argtypes = [c_void_p]
        x11.XSync.argtypes = [c_void_p, c_int]
        display = self.display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError('No se pudo abrir la pantalla X11 de RViz.')
        self.setAttribute(QtCore.Qt.WA_NativeWindow)
        self.attached = False
        attributes = XWindowAttributes(override_redirect=1)
        # Evitar que el gestor vuelva a decorar RViz como ventana independiente.
        x11.XChangeWindowAttributes(display, identifier, 1 << 9, byref(attributes))
        x11.XUnmapWindow(display, identifier)
        x11.XSync(display, 0)
        self.destroyed.connect(lambda: x11.XCloseDisplay(display))
        # Mutter retira el marco de la ventana de forma asíncrona al desmapearla.
        QtCore.QTimer.singleShot(400, self.attach_window)

    def attach_window(self):
        self.x11.XReparentWindow(self.display, self.identifier, int(self.winId()), 0, 0)
        self.attached = True
        self.resize_window(self.size())
        self.x11.XMapWindow(self.display, self.identifier)
        self.x11.XSync(self.display, 0)

    def resizeEvent(self, event):
        if self.attached:
            self.resize_window(event.size())

    def resize_window(self, size):
        self.x11.XMoveResizeWindow(self.display, self.identifier, 0, 0,
                                   max(1, size.width()), max(1, size.height()))
        self.x11.XSync(self.display, 0)


class RvizPane(QtWidgets.QWidget):
    def __init__(self, process):
        super().__init__()
        self.process = process
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.message = QtWidgets.QLabel('Abriendo RViz…', alignment=QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.message)
        self.deadline = time.monotonic() + 15
        self.timer = QtCore.QTimer(self, interval=300)
        self.timer.timeout.connect(self.attach)
        self.timer.start()

    def attach(self):
        def property_of(*args):
            return subprocess.run(['xprop', *args], capture_output=True, text=True, timeout=.4).stdout

        try:
            if self.process.poll() is not None:
                raise RuntimeError('RViz se cerró. Revisa rviz.log en la carpeta de caché.')
            for identifier in re.findall(r'0x[0-9a-f]+', property_of('-root', '_NET_CLIENT_LIST')):
                data = property_of('-id', identifier, '_NET_WM_PID', '_NET_WM_NAME', 'WM_NAME')
                if not re.search(rf'_NET_WM_PID[^\n]*=\s*{self.process.pid}\s*$', data, re.M) or '.rviz' not in data.lower():
                    continue
                self.container = RvizSurface(int(identifier, 16), self)
                self.layout.replaceWidget(self.message, self.container)
                self.message.hide()
                self.container.show()
                self.timer.stop()
                return
            if time.monotonic() >= self.deadline:
                raise RuntimeError('RViz no se pudo acoplar. Revisa la sesión X11/XWayland.')
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.message.setText(str(exc))
            self.timer.stop()


def orientation_error(a, b):
    return math.degrees(float(np.linalg.norm(rotation_vector(a @ b.T))))


def minimum_jacobian_singular_value(model, envelope, target, reference):
    """Jacobiano en m/rad con la parte angular escalada a un brazo de 0,2 m."""
    step = 1e-4
    jacobian = np.empty((6, 6))
    for axis in range(6):
        perturbed = list(target)
        perturbed[axis] += math.degrees(step)
        pose = model.tip(perturbed, envelope)
        jacobian[:3, axis] = (pose[:3, 3] - reference[:3, 3])/step
        jacobian[3:, axis] = .2*rotation_vector(pose[:3, :3] @ reference[:3, :3].T)/step
    return float(np.linalg.svd(jacobian, compute_uv=False)[-1])
