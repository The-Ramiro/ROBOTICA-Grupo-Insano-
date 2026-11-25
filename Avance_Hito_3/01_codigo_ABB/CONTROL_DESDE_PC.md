# Preparación de control desde la PC · 10 de noviembre de 2026

El monitor actual recibe ángulos y XYZ por RWS; no tiene salida de movimiento.
La coincidencia de la FK en una postura no verifica colisiones, fuerzas ni la
capacidad de detener el brazo desde una PC o una máquina virtual.

## Propuesta inicial para revisar

Interfaz con selección J1…J6, paso pequeño y signo +/−, postura medida como
origen, destino mostrado antes de ejecutar y vista previa en un modelo separado.
Una orden por vez; sin acumulación de clics ni repetición automática tras
reconectar. Una desconexión invalida cualquier orden pendiente.

`preparar_paso.py` prepara solamente una trayectoria de vista previa y rechaza
valores no finitos, tipos erróneos, datos viejos, pasos excesivos y destinos
fuera de los márgenes articulares del modelo. Conserva las vueltas de J6.
Usa una curva suave con velocidades y aceleraciones inicial/final nulas.
Los límites de demostración de ese archivo NO están aprobados para uso físico.
`send_to_robot` está bloqueado y no abre ninguna conexión.

## Datos pendientes antes de desarrollar y habilitar la salida física

- Herramienta/TCP, carga y centro de gravedad reales.
- Límites autorizados: articulaciones, velocidades, aceleraciones y zona de trabajo.
- Obstáculos: mesa, base, cables y personas; comprobar el recorrido de todos los
  eslabones, no solo la posición final del TCP.
- Estado actual de referencia/sincronización, modos y cadena de seguridad del ABB.
- Respaldo y configuración RAPID/EGM existente revisados por el laboratorio.
- Acceso físico a la parada y supervisión del responsable durante la puesta en marcha.

## Canal físico candidato

Para la integración ROS 2 se estudia EGM con el driver `abb_ros2` de Humble,
adaptado al IRB120. La configuración de ejemplo es para un IRB1200 diferente.
No copiar su programa ni sus parámetros directamente en el ABB del laboratorio.
La presencia de la opción 689-1 no demuestra que el canal esté configurado.

Antes de una orden física debe definirse y comprobarse en el controlador el
comportamiento ante pérdida de paquetes, cierre de la PC y fallo del programa.
Un watchdog o un botón de detener en Python no son una función de seguridad
certificada. Las protecciones del ABB y la parada física siguen siendo necesarias.
No cambiar automáticamente modos, encender motores ni reiniciar RAPID para recuperar conexión.

Fuentes consultadas:

- https://github.com/PickNikRobotics/abb_ros2/blob/humble/README.md
- https://github.com/PickNikRobotics/abb_ros2/blob/humble/docs/README.md
- https://github.com/PickNikRobotics/abb_ros2/blob/humble/docs/RobotStudioSetup.md

Estado: base de validación de vistas previas preparada; interfaz y canal de control
físico pendientes de definir con los datos anteriores. No se enviaron movimientos.

## Preferencias confirmadas por el usuario

Se requieren dos modos: introducir seis ángulos e introducir XYZ. Para XYZ se
prepara mantener la orientación actual del TCP. Se solicita impedir que la punta
baje y choque contra el piso. El usuario confirmó solo la brida, sin herramienta
adicional, y altura mínima de 25 cm respecto a la base: Z mínima = 0,25 m.
Se guardó en `configuracion_control.json` con desplazamiento TCP nulo.
El plano físico del piso respecto a la base sigue sin estar medido; no se supone
que coincida con Z=0. `configured_envelope` rechaza la carga mientras falte ese dato.

`objetivos_preview.py` prepara ambos tipos de objetivos, usa IK numérica con
orientación conservada para XYZ y comprueba alturas a lo largo de muestras de
una trayectoria. Usa cajas conservadoras de las mallas para detectar invasión
del plano del piso por los eslabones del modelo. Requiere alturas en el marco
base_link y desplazamiento de herramienta explícitos: no presupone que el piso
esté a z=0 ni que el TCP del notebook esté instalado en el brazo físico.

Esta comprobación discreta no certifica colisiones continuas, autocolisiones ni
protección de personas. La geometría de una herramienta adicional y los obstáculos
de la celda siguen sin estar modelados. Todas las propuestas quedan marcadas
`hardware_execution_allowed=False`. Todavía no hay interfaz ni salida física
para estos dos modos. Las cifras de los tests son sintéticas, no límites reales.

Corrección posterior del usuario: el límite es el plano horizontal a la altura
del punto de giro del eslabón 1. Falta identificar si se refiere al giro de la
base o al hombro. El límite anterior de 0,25 m queda retirado y no está activo;
min_tip_z_m es null y configured_envelope rechaza su uso hasta aclararlo.

Aclaración final: el usuario identifica el giro de la base (joint_1). El URDF
sitúa su origen en (0,0,0) de base_link, con eje vertical (0,0,1). El plano de
vista previa pasa por ese origen: Z mínima de tool0 = 0 m. No se confunde con
el hombro ni se conserva el valor de 25 cm. La altura física del cuerpo de la
base no se deduce de un eje de rotación vertical: la correspondencia de este
plano con una superficie física sigue sin haberse medido. No se habilita control
físico ni se asume que el piso coincida con el plano Z=0.
