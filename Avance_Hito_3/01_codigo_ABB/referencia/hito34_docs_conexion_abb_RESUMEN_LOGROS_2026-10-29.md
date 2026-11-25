# Qué logramos el 29 de octubre de 2026: conexión con el ABB IRB 120

Resumen de la primera sesión con el robot real del laboratorio (IRB 120-3/0.6, sistema `120-101516`). Distingue lo **medido en el robot**, lo **probado solo con simulaciones** y lo que **sigue pendiente**. Ningún comando de este día movió el robot, encendió motores ni inició programas.

## En una frase

La PC de Linux ya se comunica con el controlador por Ethernet, **lee los seis ejes reales**, y la cinemática directa del Hito 2 **coincide con la del controlador** (0,01 mm y 0,0004° en una postura). Falta ver el modelo moverse en RViz con el robot real.

## Línea de tiempo

| Etapa | Qué pasó | Evidencia |
|---|---|---|
| Preparación | Se revisó el traspaso de ChatGPT, se detectó el adaptador USB-Ethernet y se escribió la guía del día | [`GUIA_G1_DIA_LABORATORIO.md`](GUIA_G1_DIA_LABORATORIO.md) |
| **G1-A**, enlace | Con el cable en el puerto **X2 Service**, el controlador entregó IP por DHCP; ping 3/3 | [`g1a_enlace_2026-10-29.json`](evidencias/g1a_enlace_2026-10-29.json) |
| **G1-B**, lectura RWS | Con la cuenta de fábrica documentada por ABB se leyó identidad, opciones y los seis ejes | [`g1_rws_2026-10-29_sanitizado.json`](evidencias/g1_rws_2026-10-29_sanitizado.json) |
| Contraste con el FlexPendant | Los ejes de la pantalla coinciden con la lectura RWS | [`g1b_contraste_flexpendant_2026-10-29.json`](evidencias/g1b_contraste_flexpendant_2026-10-29.json) |
| **G2 parcial** | La FK del modelo DH coincide con la posición cartesiana del controlador | [`g2_fk_vs_controlador_pose1_2026-10-29.json`](evidencias/g2_fk_vs_controlador_pose1_2026-10-29.json) |
| Puente a ROS 2 | Los ejes reales se publican como `JointState` (probado contra un controlador simulado) | [`GUIA_PUENTE_ROS2.md`](GUIA_PUENTE_ROS2.md) |
| Gemelo en RViz | El modelo se mueve con los ejes publicados (probado contra un controlador simulado) | [`GUIA_PUENTE_ROS2.md`](GUIA_PUENTE_ROS2.md) |

## Resultados medidos en el robot real

| Medida | Resultado |
|---|---|
| Enlace | Cable en X2, IP de la PC `192.168.125.203/24` por DHCP del controlador, ruta por Ethernet y Wi-Fi intacto |
| Ping a `192.168.125.1` | 3 de 3 respuestas, 0 % de pérdida, 0,8 ms |
| Identidad por RWS | Sistema `120-101516`, RobotWare **6.12.01.00** (confirma lo transcrito del FlexPendant), opción `IRB 120-3/0.6` y *Drive System IRC5 Compact* |
| Opciones | 689-1 EGM, 616-1 PC Interface, 623-1 Multitasking y UDPUC Driver presentes |
| Estado del controlador | `motoroff`, modo `AUTO`, RAPID `stopped`, `ROB_1` activada |
| Ejes: RWS frente a FlexPendant | Diferencia máxima **0,004°** en los seis (resolución de la pantalla 0,01°) |
| `tool0` en `wobj0`: modelo frente a controlador | Diferencia de **0,01 mm** y **0,0004°** (los límites de resolución de la pantalla) |

Esa postura habría detectado cualquier inversión de signo, 1° de offset en una articulación o 1 mm de error en `d1`, `a2`, `a3`, `d4` o `d6` (calculado a propósito).

## Resultados medidos solo con simulaciones

| Medida | Resultado | Qué falta |
|---|---|---|
| Puente RWS → ROS 2 | 15 mensajes a 5,01 Hz, `joint_1…joint_6` en radianes correctos | Ejecutarlo con el robot real |
| Gemelo: transformadas de ROS frente a la FK del Hito 2 | **0,0000 mm** en 76 comparaciones; `tool0` recorrió x 119–495, y −334–413, z 540–709 mm | Confirmar a simple vista que el modelo de RViz se mueve con el robot |
| RViz | Arranca sobre la pantalla real sin errores (OpenGL 4.6) y se detiene sin dejar procesos | Lo mismo con el robot real |

## Qué se construyó

| Archivo | Para qué sirve |
|---|---|
| `scripts/g1_verificar_enlace.py` | Comprueba adaptador, cable, IP y ruta; solo lee el estado local |
| `scripts/abb_lectura_rws.py` (ya existía) | Lectura RWS de solo lectura, usada hoy con el robot |
| `hito2/comparar_lectura_abb.py` | Calcula qué predice el modelo para unos ejes y lo compara con el FlexPendant (posición y cuaternión) |
| `scripts/abb_rws_joint_bridge.py` | Puente de solo lectura RWS → `JointState` |
| `scripts/escuchar_joint_states.py` | Muestra lo que recibe un nodo ROS (sustituye a `ros2 topic echo`, que no funciona aquí) |
| `scripts/gemelo_rviz.sh` y `scripts/preparar_gemelo.py` | Levantan `robot_state_publisher`, RViz y el puente juntos |
| `scripts/tests/…` y `hito2/tests/test_comparar_lectura_abb.py` | 42 pruebas offline en `scripts/tests` más las del comparador |

El puente no tiene suscriptores, servicios ni acciones: nada de ROS llega al controlador, y una prueba lo verifica.

## Lo que NO se ha demostrado

- **Una sola postura validada contra el controlador.** La segunda lectura (`~/lecturas_abb/postura_B.json`) está tomada, pero falta la foto del FlexPendant para compararla. Una validación sólida pide 3 o 4 posturas más que muevan J3, J6 y J5 positivo.
- **El TCP de 0,150 m no está verificado en el robot.** Se comparó `tool0`.
- **El puente y el gemelo no se han ejecutado con el robot real**, y nadie ha confirmado a simple vista que el modelo de RViz se mueva.
- **No hay validación de la dinámica 1R, del LSPB ni de las singularidades** contra el robot.
- **EGM y control:** no se han configurado ni probado. Requieren cambios en el controlador, un respaldo del sistema y el visto bueno explícito del docente y de quien opere el laboratorio.
- **Se trabajó sin operador presente.** El selector estaba en `AUTO` con motores apagados, y la pantalla mostraba el aviso `90216 Enabling device active in Auto mode`.

## Errores y correcciones del día

- Se ejecutó un comando con números de ejemplo (que eran el valor del modelo redondeado) como si fueran del FlexPendant, y salió una "diferencia" de 0,07 mm sin valor. Se descartó, y la validación válida usa los números leídos de la foto.
- Se dijo que el campo `starttm` de `/rw/system` era la hora de arranque. Dos lecturas separadas por 23 min 52 s mostraron que sigue al reloj del controlador (que va 1 h 03 min por delante de UTC).
- El comando de pruebas documentado falló con Python 3.14 y se corrigió agregando `-t scripts/tests`.
- El archivo de lectura original incluye la MAC y la IP de la PC: quedó fuera de git y se subió una copia sanitizada.

## Entorno

Ubuntu 26.04 con Python 3.14, pero ROS 2 Jazzy está compilado para Python 3.12. Con el Python por defecto `rclpy`, `ros2` y `colcon` no funcionan. Se creó `~/ros_py312` (Python 3.12 con `numpy`, `pyyaml` y `requests`), fuera del repositorio. Con él funcionan los nodos de este día; **no** funcionan `ros2 topic`, `ros2 launch` ni `colcon build`.

## Cómo reproducirlo

Desde `proyecto/`, con el cable en X2:

```bash
python3 scripts/g1_verificar_enlace.py --ping
python3 scripts/abb_lectura_rws.py --host 192.168.125.1 --user "Default User" --output ~/lecturas_abb/lectura.json
scripts/gemelo_rviz.sh --host 192.168.125.1 --user "Default User"
```

Las pruebas offline: `python3 -m unittest discover -s scripts/tests -t scripts/tests`.

## Siguientes pasos

1. Foto del FlexPendant en modo *Linear* para cerrar la comparación de la segunda postura, y 3 o 4 posturas más.
2. Ejecutar el gemelo con el robot real mientras alguien lo mueve, y confirmar que el modelo sigue.
3. Resolver el entorno (compilar el paquete fuera de `colcon`, o usar Docker con Jazzy sobre Ubuntu 24.04).
4. Control por EGM, solo con respaldo del sistema y GO explícito.
