"""Configuración explícita de la conexión y del módulo del controlador."""
import ipaddress
from pathlib import Path

from PyQt5 import QtWidgets


def connection_settings(parent, host='192.168.125.1', user='Default User'):
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle('Conectar al ABB')
    dialog.setMinimumWidth(520)
    layout = QtWidgets.QVBoxLayout(dialog)
    description = QtWidgets.QLabel(
        'Al aceptar se detiene RAPID en curso, se carga y comprueba el módulo elegido '
        'y se habilita la recepción automática al 0% de velocidad. '
        '100% corresponde a vmax de ABB. Elige y aplica un porcentaje antes de Ejecutar.')
    description.setWordWrap(True)
    layout.addWidget(description)
    form = QtWidgets.QFormLayout()
    ip = QtWidgets.QLineEdit(host)
    username = QtWidgets.QLineEdit(user)
    password = QtWidgets.QLineEdit()
    password.setEchoMode(QtWidgets.QLineEdit.Password)
    password.setPlaceholderText('Vacía usa el acceso de fábrica del ABB')
    filename = QtWidgets.QLineEdit(str(Path(__file__).with_name('rapid')/'IMT342_Automatico.mod'))
    browse = QtWidgets.QPushButton('Elegir…')
    def choose_file():
        selected, _ = QtWidgets.QFileDialog.getOpenFileName(
            dialog, 'Seleccionar módulo automático', filename.text(), 'Módulos RAPID (*.mod)')
        if selected:
            filename.setText(selected)
    browse.clicked.connect(choose_file)
    file_row = QtWidgets.QHBoxLayout()
    file_row.addWidget(filename, 1)
    file_row.addWidget(browse)
    form.addRow('IP del ABB', ip)
    form.addRow('Usuario', username)
    form.addRow('Contraseña', password)
    form.addRow('Archivo RAPID', file_row)
    layout.addLayout(form)
    hint = QtWidgets.QLabel('El controlador debe estar en AUTO con sus protecciones habilitadas.')
    hint.setWordWrap(True)
    layout.addWidget(hint)
    error = QtWidgets.QLabel()
    error.setWordWrap(True)
    layout.addWidget(error)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
    def accept():
        try:
            address = ipaddress.IPv4Address(ip.text().strip())
            if address.is_multicast or address.is_unspecified or int(address) & 255 == 255:
                raise ValueError('Escribe una IP IPv4 válida para el ABB.')
            if not username.text().strip():
                raise ValueError('Falta el usuario del ABB.')
            if not Path(filename.text()).is_file():
                raise ValueError('Selecciona el archivo IMT342_Automatico.mod.')
        except ValueError as exc:
            error.setText(str(exc))
            return
        dialog.accept()
    buttons.accepted.connect(accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    if dialog.exec_() != QtWidgets.QDialog.Accepted:
        password.clear()
        return None
    result = (ip.text().strip(), username.text().strip(), password.text() or 'robotics', filename.text())
    password.clear()
    return result
