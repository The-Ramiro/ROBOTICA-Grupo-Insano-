# ABB real en RViz · Ubuntu 22.04 / ROS 2 Humble

El acceso actual `abrir_panel.sh` abre **Automático**, deshabilitado hasta
pulsar **Conectar** y aceptar la IP y el archivo del controlador. El módulo preparado es
`rapid/IMT342_Automatico.mod`. Los pasos y las comprobaciones pendientes están
en [AUTOMATICO.md](AUTOMATICO.md).

El panel actual `panel_robotica.py` recupera **Ejecutar** e incluye pose 6D,
anillos de rotación, cinemática, LSPB y estimación dinámica. Los destinos se
envían al pulsar **Ejecutar**. Los perfiles LSPB sirven para el análisis y el
controlador ABB conserva su planificación interna de MoveAbsJ.

La versión manual se abre con `abrir_manual.sh`. La copia completa está en
`IMT342/versiones/ABB_manual_2026-11-18`.

## Interfaz manual: brazo real y sombra de destino

El acceso manual (`bash ~/abb-real/abrir_manual.sh`) abre
`panel_sombra.py`. Sustituye la interfaz de reproducción descrita más abajo.

- **Naranja:** exclusivamente los seis ángulos recibidos del controlador real.
  No se publica una postura de muestra como si fuera real. Sin la primera
  lectura, el modelo naranja no se inicia. Si se pierde la conexión después,
  conserva la última postura y se identifica como dato anterior/congelado.
- **Blanco semitransparente:** destino propuesto. Los deslizadores lo colocan
  directamente, sin animar una trayectoria. XYZ actualiza solo ese destino y
  conserva la orientación actual de la sombra.
- **Poner sombra sobre el brazo real** en **Control** copia una lectura reciente válida a los
  controles del destino, sin enviar movimientos.
- **Envío desde IK:** al soltar la esfera o pulsar Enter en XYZ, se calcula el
  destino, se revisan los seis ángulos en Ubuntu y se envía una orden a
  `IMT342_DestinoDirecto / ServidorDirecto` mediante RWS. El operador mantiene el
  habilitador en manual reducido y concede el acceso remoto RMMP si lo pide el
  controlador. La rutina RAPID ejecuta al recibir la orden válida.
  Se comprueban identidad, módulo, puntero, postura de partida, límites y altura
  prevista de la brida. El módulo ejecuta un solo `MoveAbsJ`. Los registros están
  en `~/abb-real/ordenes/`. Consultar [el protocolo RAPID](rapid/LEEME.md).

Los dos robots se superponen en el mismo marco `world`, con árboles TF separados
`real/` y `goal/`, temas de estado distintos y descripciones de robot distintas.
Mover el blanco no publica en el tema del naranja ni en un driver del ABB.
El porcentaje de velocidad se ajusta en el FlexPendant. Para detener un
movimiento, el operador usa el habilitador o Stop del FlexPendant.

El panel integra RViz mediante Qt y X11/XWayland. Requiere `xprop` del paquete
`x11-utils`. **Control** reúne los seis ángulos y los tres campos del destino.
Al cambiar un ángulo, la cinemática directa actualiza XYZ. Al editar XYZ, la
cinemática inversa calcula los ángulos y conserva la orientación de la brida.
Con **Interact**, arrastra el punto turquesa de la brida blanca en el plano de
la vista o usa sus flechas para moverlo por eje. Con **Publish Point**, marca un
punto sobre una superficie visible en RViz. Al soltar la esfera fuera del
espacio alcanzable, regresa a la brida en la última postura válida.
**Verificación** muestra los ángulos y XYZ medidos junto al destino y su
diferencia, el error entre FK y el XYZ del controlador y el residual entre
el punto pedido y el obtenido por IK.
Para la pose objetivo, muestra las ramas geométricas del IRB 120 que cumplen
los rangos y la altura configurados. Incluye vueltas equivalentes de J6 cuando
caben en sus límites. Seleccionar una fila muestra el residual de posición y
orientación, el Jacobiano y el margen articular. Para explorar el mismo punto
con otra orientación, se muestran seis muestras de ±30° alrededor de los ejes
locales de la brida. Las familias singulares se muestran con ejemplos
representativos. La tabla analiza configuraciones de llegada. La comprobación
del recorrido se realiza antes de enviar la orden.
El Jacobiano numérico usa una escala angular de 0,2 m.
Diseño de referencia: [panel del equipo](https://github.com/Marcelo767n/robotica-IMT342/tree/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/ros2_ws/src/irb120_rviz_panel).

La versión actual de `ServidorDirecto` requiere cargar y comprobar su `.mod` en
RobotWare 6.12. El panel verifica la huella antes de enviar un destino.

## Panel didáctico anterior

Abre **ABB panel didactico** en Aplicaciones o ejecuta
`bash ~/abb-real/abrir_panel.sh`.

1. **Ángulos:** seis deslizadores horizontales, escritura en grados y porcentaje
   de posición dentro del rango de cada eje. Cambian solo el modelo de exploración.
2. **XYZ:** escritura en mm respecto a la base, conservando la orientación del
   inicio. Se calcula una trayectoria articular hasta ese destino; no se promete
   una línea recta del TCP.
3. **Velocidad y límites:** deslizador porcentual, tabla de máximos ABB y límites
   elegidos en °/s, desplazamiento máximo por orden y acceso al editor por eje.
4. **Preparar trayectoria** calcula el recorrido y duración con esos límites.
   **Reproducir vista previa** anima el modelo con el tiempo calculado.
5. **Ejecutar en el brazo real** está visible y bloqueado: no hay envío físico
   implementado. No se presenta un botón simulado como si controlara el ABB.

El inicio predeterminado es una postura de muestra, identificada como tal.
**Conectar lectura del ABB** usa solo GET. Después, **Usar postura real como inicio**
copia una lectura reciente y válida sin mover el robot. Una referencia fuera de
los límites no se recorta ni se sustituye silenciosamente.

La vista previa usa el dominio ROS 45; el monitor real continúa en el 44.
No conecta consignas de ROS al ABB. El modelo de exploración no representa por
sí mismo la postura real. El plano mínimo de la brida es Z=0 del modelo.
Como el piso físico sigue sin medirse, esta exploración no verifica colisiones
con ese piso ni con los obstáculos del laboratorio. La salida física queda bloqueada.

Si una propuesta grande supera el recorrido máximo por orden, será rechazada.
Ese porcentaje puede ajustarse expresamente en la pestaña de velocidad. Cambiar
un objetivo o límite invalida la trayectoria preparada. No se reanuda la animación
sola después de un cambio ni se envían comandos tras reconectar.

## Límites y porcentajes

Abre **ABB limites y porcentajes** en Aplicaciones o pulsa **Configurar limites
y porcentajes** desde el monitor del robot. También puedes ejecutar
`bash ~/abb-real/configurar_limites.sh`.

Permite editar el rango mínimo/máximo de cada articulación, la velocidad global,
el porcentaje de velocidad de cada eje y el desplazamiento máximo por orden.
Los valores no pueden ampliar los rangos de la ficha ABB del IRB120 estándar.
No cambia parámetros en el controlador físico. El perfil se guarda localmente
en `parametros_movimiento.json` y lo utiliza `prepare_target` al preparar movimientos.

- Velocidad efectiva = máximo ABB × porcentaje global/100 × porcentaje del eje/100.
- Rango usado (%) = 100 × (máximo configurado − mínimo configurado)/(máximo ABB − mínimo ABB).
- Recorrido por orden (%) = 100 × cambio angular máximo permitido/(amplitud completa ABB).
- Posición dentro del rango: 0% es el mínimo ABB y 100% el máximo; se ve en la ayuda del rango.

La velocidad mostrada es un límite configurado, no velocidad real medida ni
el override del FlexPendant. El valor inicial global es 1%, editable; no implica
que sea seguro para la celda. 0% impide preparar una trayectoria con desplazamiento.
Se conserva un margen articular y el filtro de altura. La aceleración es un valor
de simulación en °/s²; no se inventa un porcentaje máximo de aceleración de ABB.

Referencia: ficha ABB IRB120 Rev. J, noviembre de 2019:
https://library.e.abb.com/public/7aa0711a20fa41c49a8fdf3bbc3d5bb0/IRB120-Rev.J-ROBO149EN_D.pdf

J1: −165…165°, 250°/s; J2: −110…110°, 250°/s;
J3: −110…70°, 250°/s; J4: −160…160°, 320°/s;
J5: −120…120°, 320°/s; J6 estándar: −400…400°, 420°/s.
La configuración real del ABB puede imponer límites adicionales que todavía
no se importaron. La salida física continúa bloqueada.

Abre **ABB real en RViz** en Aplicaciones, o desde tu carpeta personal:

```bash
bash ~/abrirabb_real
```

IP: `192.168.125.1`. Usuario: `Default User`. Si la cuenta sigue con la
configuración de fábrica documentada por ABB, deja la contraseña vacía y pulsa
OK; se utiliza `robotics`. Si la cuenta cambió, escribe su contraseña actual.
No se guardan contraseñas ni cookies en disco.

Conecta la computadora al puerto de servicio X2 del ABB usando el enlace que ya
emplearon para leer sus ejes. Ubuntu debe poder alcanzar esa IP. El programa no
cambia la configuración de red de Windows, VirtualBox ni del robot.

Cuando el operador mueva el robot con el FlexPendant, los seis ángulos recibidos
se convierten de grados a radianes y actualizan el modelo en RViz. La ventana
indica EN VIVO, la antigüedad del dato y la frecuencia observada. Cierra esa
ventana para detener todo. Si se corta la conexión, se conserva la última postura,
se muestra el aviso y se reintenta automáticamente. No se fabrican lecturas de cero.

La pantalla muestra dos posiciones XYZ, en mm:

- **ABB tool0 / Base**: lectura del controlador, aproximadamente cada 0,5 s.
- **Modelo tool0 / base_link**: posición calculada con el mismo URDF de RViz.

Ambas corresponden a la brida `tool0`; no incluyen el TCP adicional de 150 mm
del notebook. No deben compararse con un TCP distinto seleccionado en el pendant.
Las lecturas articulares y cartesianas son sucesivas, no una captura simultánea.

## Errores en pantalla

Cada eje muestra el ángulo real, una referencia editable en grados, el error
con signo `referencia - real` y su valor absoluto. No se reducen vueltas de J6.
Puedes escribir una postura objetivo o pulsar **Tomar postura actual como
referencia** para medir después cuánto se aparta de esa postura. Estas referencias
solo cambian la comparación en pantalla: no se envían al robot. Sin referencia
o sin lectura reciente, el error se muestra con un guion.

Para la posición se calcula `delta = FK_URDF(q_real) - XYZ_ABB` y la distancia
`sqrt(deltaX² + deltaY² + deltaZ²)`, en mm. Se conservan los ángulos asociados a
la consulta cartesiana, y se muestra la separación temporal entre ambas lecturas.
Compara en reposo: durante el movimiento esa separación puede producir diferencias.
Es una comparación entre el modelo y la cinemática del controlador; no mide la
precisión física del robot frente a un instrumento externo. Un 0,000 mm es un
resultado redondeado a tres decimales, no una prueba de error exactamente nulo.

Actualización articular solicitada: hasta 10 Hz, limitada por HTTP y la red.
Es visualización continua por sondeo RWS; no es un lazo de control de tiempo real.
No hace falta iniciar EGM ni RAPID. El programa solo consulta recursos GET y no
envía movimientos, cambios de modo ni comandos de motores.

## Copiar a otro Ubuntu con Humble

Extrae esta carpeta y ejecuta:

```bash
bash instalar_acceso.sh
bash abrir_real.sh
```

Utiliza las dependencias del visualizador que ya teníamos: Python del sistema,
rclpy, sensor_msgs, PyQt5, numpy, yaml, robot_state_publisher y rviz2. Si falta
alguna, en Ubuntu 22.04 con ROS Humble instalado:

```bash
sudo apt install python3-pyqt5 python3-numpy python3-yaml ros-humble-rclpy ros-humble-sensor-msgs ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-interactive-markers
```

Se usa el dominio ROS 44 y el tema `/irb120/real/joint_states`; el visualizador
local anterior utiliza el dominio 43. No se conecta ningún suscriptor de comandos
al controlador. El acceso anterior `abrirabb` continúa siendo el modelo manual.

## Base utilizada

Se encontró el resumen en **proyectoHito3y4**, no en el enlace inicial de Hito 2:

https://github.com/Marcelo767n/robotica-IMT342/blob/888abe7134d536f29b71eb9031dd2f54a354f169/2-2026/IMT342/equipo/proyecto/docs/conexion_abb/RESUMEN_LOGROS_2026-10-29.md

Se reutilizan las funciones del lector y del puente de esa revisión, copiadas sin
cambios en `abb_lectura_rws.py` y `abb_rws_joint_bridge.py`. `abb_real.py` agrega
la interfaz, consultas en segundo plano, lectura cartesiana y arranque conjunto
adaptado a Humble. Las copias de referencia están en `referencia/`.

Modelo y mallas: el mismo ROS-Industrial IRB120 del visualizador previo;
licencia y procedencia incluidas. No se alteraron dimensiones ni calibración.

API cartesiana oficial:
https://developercenter.robotstudio.com/api/rwsApi/ms_mechunit_robtarget_page.html

Estado de entrega: preparado para probar seguimiento con el robot en laboratorio.
La evidencia de lectura física es la del repositorio; no se afirma haber probado
esta integración con el brazo físico desde esta computadora.
