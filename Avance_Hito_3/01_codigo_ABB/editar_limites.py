#!/usr/bin/env python3
"""Editor local. No importa ROS ni abre conexiones con el robot."""
import sys
from PyQt5 import QtWidgets, QtCore
from limites_movimiento import FACTORY, load, save, effective, angle_percent


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QWidget()
    win.setWindowTitle('ABB IRB120 | Limites y porcentajes · vista previa')
    win.resize(1120, 540)
    layout = QtWidgets.QVBoxLayout(win)
    layout.addWidget(QtWidgets.QLabel('<h2>Limites configurables · IRB120-3/0.6</h2>'))
    config = load()
    top = QtWidgets.QHBoxLayout()
    top.addWidget(QtWidgets.QLabel('Velocidad global [% del maximo ABB]'))
    global_speed = QtWidgets.QDoubleSpinBox()
    global_speed.setRange(0, 100)
    global_speed.setDecimals(2)
    global_speed.setValue(config['global_speed_percent'])
    top.addWidget(global_speed)
    top.addWidget(QtWidgets.QLabel('Aceleracion de simulacion [°/s²]'))
    acceleration = QtWidgets.QDoubleSpinBox()
    acceleration.setRange(.01, 100)
    acceleration.setValue(config['preview_acceleration_deg_s2'])
    top.addWidget(acceleration)
    layout.addLayout(top)
    table = QtWidgets.QTableWidget(6, 9)
    table.setHorizontalHeaderLabels(['Eje', 'Rango ABB [°]', 'Min [°]', 'Max [°]', 'Rango usado [%]',
                                     'Vel. eje [%]', 'Vel. efectiva', 'Por orden [%]', 'Por orden [°]'])
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
    table.verticalHeader().setVisible(False)
    controls = []
    for i, (row, (lo, hi, speed)) in enumerate(zip(config['axes'], FACTORY)):
        for col, label in ((0, str(i+1)), (1, f'{lo:g} a {hi:g}'), (4, ''), (6, ''), (8, '')):
            item = QtWidgets.QTableWidgetItem(label)
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
            table.setItem(i, col, item)
        fields = []
        for col, key, lower, upper in ((2, 'min_deg', lo, hi), (3, 'max_deg', lo, hi),
                                       (5, 'axis_speed_percent', 0, 100),
                                       (7, 'max_command_travel_percent', .01, 100)):
            field = QtWidgets.QDoubleSpinBox()
            field.setRange(lower, upper)
            field.setDecimals(2)
            field.setValue(row[key])
            table.setCellWidget(i, col, field)
            fields.append(field)
        controls.append(fields)
    layout.addWidget(table)
    explanation = QtWidgets.QLabel('Velocidad efectiva = maximo ABB × velocidad global % × velocidad del eje %.\n'
        'Rango usado = amplitud configurada / amplitud ABB. Por orden limita el cambio total de cada eje.\n'
        'Se conserva el margen articular de 2° y el filtro de altura. Un porcentaje menor no garantiza evitar choques.\n'
        'Estos parametros afectan la preparacion de trayectorias; no cambian limites del controlador ABB.\n'
        'Aceleracion: parametro provisional de simulacion, sin porcentaje de fabricante verificado.')
    explanation.setWordWrap(True)
    layout.addWidget(explanation)
    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    layout.addWidget(status)
    button = QtWidgets.QPushButton('Guardar limites para vista previa')
    layout.addWidget(button)

    def current():
        result = dict(config)
        result['global_speed_percent'] = global_speed.value()
        result['preview_acceleration_deg_s2'] = acceleration.value()
        result['axes'] = [dict(zip(('min_deg', 'max_deg', 'axis_speed_percent', 'max_command_travel_percent'),
                                  [field.value() for field in row])) for row in controls]
        return result

    def refresh():
        try:
            rows = effective(current())
            for i, row in enumerate(rows):
                table.item(i, 4).setText(f"{row['range_percent']:.2f}%")
                table.item(i, 6).setText(f"{row['speed_deg_s']:.3f} °/s ({row['speed_percent']:.2f}%)")
                table.item(i, 8).setText(f"{row['max_command_delta_deg']:.3f}°")
                lo = controls[i][0].value()
                hi = controls[i][1].value()
                table.item(i, 4).setToolTip(f'Extremos: {angle_percent(i,lo):.2f}% a {angle_percent(i,hi):.2f}% del rango ABB')
            status.setText('Perfil valido para vista previa. Envio fisico BLOQUEADO.')
            button.setEnabled(True)
            return True
        except (ValueError, KeyError, TypeError) as exc:
            status.setText(str(exc))
            button.setEnabled(False)
            for i in range(6):
                for col in (4, 6, 8):
                    table.item(i, col).setText('—')
            return False

    def store():
        if refresh():
            save(current())
            status.setText('Limites guardados. No se envio ninguna orden al ABB.')
    for fields in controls:
        for field in fields:
            field.valueChanged.connect(refresh)
    global_speed.valueChanged.connect(refresh)
    acceleration.valueChanged.connect(refresh)
    button.clicked.connect(store)
    refresh()
    win.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
