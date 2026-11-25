# Abrir el panel

Entorno del proyecto: Ubuntu 22.04 de escritorio con ROS 2 Humble. Conserva la estructura de esta carpeta. Descomprime en la carpeta personal de Ubuntu y ejecuta desde aquí:

```bash
bash ./abrir_panel.sh
```

El panel se abre para inspeccionar el modelo y los cálculos sin conectar al controlador. Usa `abrir_manual.sh` para la interfaz manual. Si faltan dependencias, revisa las listas de paquetes en `LEEME.md` y `AUTOMATICO.md`. Se utilizan RViz, robot_state_publisher, sensor_msgs, interactive_markers, rclpy y Python con PyQt5, NumPy, PyYAML y Requests.

El punto de entrada actual es `panel_robotica.py`. Los documentos y el manifiesto dentro de `referencia/` y `rws_equipo/` describen el aporte previo del equipo; el manifiesto conserva la configuración de su proyecto de origen. Para esta entrega usa el lanzador, la configuración y los módulos RAPID de la raíz. `rapid/LEEME.md` describe las versiones EGM previas. El montaje del marco en RViz es una referencia visual.

Las pruebas Python se ejecutan desde esta carpeta con `python3 -m unittest discover`.
