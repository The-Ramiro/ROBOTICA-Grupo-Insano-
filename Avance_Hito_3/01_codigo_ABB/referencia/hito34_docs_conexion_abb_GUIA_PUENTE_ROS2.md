# Puente de estado RWS → ROS 2 (solo lectura)

**Prueba preparada el 30/10:** para abrir juntos RViz y un panel con los ángulos RWS, ROS y del gemelo TF, usar la [prueba de comunicación en vivo](PRUEBA_COMUNICACION_EN_VIVO.md). Incluye frecuencia, detección de datos antiguos y registro CSV con un solo comando.

Segunda etapa de la comunicación, después de G1. **No mueve el robot**: cada ciclo hace una consulta GET de los seis ejes y los publica como `sensor_msgs/JointState`. El nodo no tiene suscriptores, servicios ni acciones, así que nada de ROS llega al controlador (una prueba lo verifica).

## Qué se comprobó y qué no

| Comprobado el 29/10 | Cómo |
|---|---|
| El puente lee, convierte grados a radianes y publica `joint_1…joint_6` | Contra un controlador **simulado** en `127.0.0.1` con autenticación Digest: 15 mensajes a 5,01 Hz recibidos por un suscriptor independiente |
| Solo hace un GET a `/rw/motionsystem/mechunits/ROB_1/jointtarget` por ciclo | Pruebas automáticas (36 en total en `scripts/tests`) |
| Rechaza frecuencias mayores de 10 Hz, direcciones multicast/broadcast y puertos ≠ 80 salvo en loopback | Pruebas automáticas |
| **Con el robot real** | **Todavía no se ha ejecutado.** Es el siguiente paso |

## Por qué se usa un entorno de Python 3.12

Esta PC tiene Ubuntu 26.04 y Python 3.14 por defecto, pero ROS 2 Jazzy está compilado para Python 3.12. Con el Python por defecto `rclpy` no carga, y los comandos `ros2` y `colcon` tampoco funcionan. Solución sin tocar el sistema: un entorno virtual **fuera del repositorio**, creado el 29/10:

```bash
python3.12 -m venv ~/ros_py312 && ~/ros_py312/bin/pip install numpy pyyaml requests
```

Con él, `rclpy` y `sensor_msgs` funcionan. **No funciona** `ros2 topic echo`, `ros2 launch` ni `colcon build`; por eso hay un escuchador propio.

## Cómo ejecutarlo con el robot

Con el cable conectado y G1 funcionando (`python3 scripts/g1_verificar_enlace.py --ping` en verde). Desde la carpeta `proyecto/`, **terminal 1**: el puente. La contraseña la pide sin mostrarla.

```bash
source /opt/ros/jazzy/setup.bash && ~/ros_py312/bin/python scripts/abb_rws_joint_bridge.py --host 192.168.125.1 --user "Default User"
```

Debe imprimir `Leyendo http://192.168.125.1 (ROB_1) a 5 Hz -> /irb120/real/joint_states. SOLO LECTURA.` y, cada segundo, los seis ejes en grados. **Si alguien mueve el robot, los valores cambian.**

**Terminal 2**, para ver el lado ROS (es lo que vería cualquier nodo):

```bash
source /opt/ros/jazzy/setup.bash && ~/ros_py312/bin/python scripts/escuchar_joint_states.py --seconds 15
```

Debe terminar con `mensajes recibidos` (unos 75 en 15 s) y `frecuencia observada: ~5 Hz`.

Detén el puente con `Ctrl+C`.

## Si falla

| Mensaje | Qué significa |
|---|---|
| `Lectura fallida … BridgeError: HTTP 401` | Usuario o contraseña incorrectos. No insistas con variaciones |
| `Lectura fallida … ConnectionError / Timeout` | Se perdió el enlace. Repite `g1_verificar_enlace.py` |
| `Ejes fuera de los limites del datasheet` | Aviso; revisa el FlexPendant |
| `ModuleNotFoundError: rclpy` | Falta `source /opt/ros/jazzy/setup.bash` o se usó otro Python que `~/ros_py312/bin/python` |
| `Lectura recuperada tras N fallos` | El enlace volvió; no requiere acción |

## Gemelo en RViz: el modelo se mueve con los ejes del robot real

Mueves el robot con el FlexPendant, el controlador reporta los ángulos, el puente los lee y RViz mueve el modelo. (El FlexPendant no envía nada a la PC por sí mismo: los ángulos salen del controlador por RWS.) Un solo comando levanta `robot_state_publisher`, RViz y el puente; la contraseña la pide al arrancar:

```bash
scripts/gemelo_rviz.sh --host 192.168.125.1 --user "Default User"
```

Se detiene todo con `Ctrl+C`. Genera sus archivos en `~/ros_twin` (fuera del repositorio) con las mallas como rutas absolutas, porque `colcon` no funciona. Lee a 10 Hz y fuerza el modo `xcb` de Qt, necesario para RViz en una sesión Wayland.

**Qué debes ver:** la ventana de RViz con el IRB 120 y sus marcos. Al mover el robot, el modelo sigue la misma postura en el mismo sentido. Si queda inmóvil, mira en la terminal si el puente imprime `Lectura fallida`.

| Comprobado el 29/10 (controlador **simulado** en movimiento) | Resultado |
|---|---|
| Transformadas de ROS (`base_link` → `tool0`) frente a la FK del Hito 2 | 0,0000 mm en 76 comparaciones |
| RViz sobre la pantalla real | Arranca sin errores; OpenGL 4.6 |
| Cierre | No quedan procesos sobrantes |
| **Con el robot real** | **Pendiente.** Nadie ha confirmado a simple vista que el modelo se mueva |

Si RViz abre pero se ve vacío: en *Displays* confirma que *Fixed Frame* es `base_link` y que el tema de `RobotModel` es `/robot_description`.

## Lo que sigue (no hecho)

1. **Control (G3).** Pasar de leer a mandar requiere EGM: configurar RAPID y UDP en el controlador, adaptar `abb_ros2` al IRB 120 y un **GO explícito** del docente y de quien opere el laboratorio, con un respaldo del sistema. Es una etapa distinta y con riesgo real.
2. **Entorno.** Compilar `irb120_hito1` fuera de `colcon` (o usar Docker con Jazzy sobre Ubuntu 24.04) para poder usar `ros2 launch` y el resto de herramientas.
