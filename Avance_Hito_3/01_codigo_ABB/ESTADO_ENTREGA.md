# Entrega · 3 de noviembre de 2026

Instalado en UBUNTU2: `/home/jose/abb-real`.
Acceso en Aplicaciones: **ABB real en RViz**.
Comando: `bash ~/abrirabb_real`.

Se abrió la ventana de conexión y después el monitor y RViz desde este acceso.
En esta sesión no hubo respuesta de `192.168.125.1`: el monitor mostró timeout y
SIN DATOS RECIENTES. No se recibieron articulaciones ni XYZ reales en esta PC.
El seguimiento con el robot queda listo para ejecutar en el laboratorio.
No se realizaron movimientos ni cambios de parámetros del controlador.

La lectura física previa del repositorio corresponde al sistema 120-101516 y
RobotWare 6.12.01.00. El resumen encontrado está en proyectoHito3y4, revisión
888abe7134d536f29b71eb9031dd2f54a354f169. Se conservó el visualizador manual.

## Actualización · 5 de noviembre de 2026

Se instaló la tabla de referencias y errores por articulación y la comparación
cartesiana modelo - ABB. Se observaron lecturas físicas en vivo a aproximadamente
9,5 Hz. Postura de la captura: [9,823; 7,160; 9,397; -6,857; 28,640; 35,511] grados.
ABB y modelo mostraron XYZ = [388,81; 63,14; 488,08] mm; diferencia total mostrada
0,000 mm (redondeada), con separación entre consultas de 14 ms. Esto indica
coincidencia cinemática en esa postura, no precisión física absoluta.
Las referencias articulares se dejaron vacías para que el usuario las introduzca.
No se enviaron consignas de movimiento. Se conservó una copia anterior en Ubuntu:
/home/jose/abb-real/abb_real.antes_errores.py.

## Panel didactico · 10 de noviembre de 2026

Preparado panel_didactico.py y abrir_panel.sh, con seis deslizadores, entrada XYZ,
velocidad porcentual y datos ABB, preparacion y reproduccion de trayectorias.
El boton de ejecucion fisica existe y permanece deshabilitado; no hay backend de
movimiento real. El usuario informa que el laboratorio ya tiene RAPID/EGM, pero
su modulo/configuracion/IP/puerto siguen sin identificarse en esta sesion.

Comprobacion local offscreen con ROS/red sustituidos por dobles: widget, slider,
preparacion articular, animacion, invalidacion por cambio de velocidad, IK XYZ y
bloqueo del boton fisico comprobados. Doce pruebas del planificador y limites pasan.
No equivale a prueba con RViz en Ubuntu ni con el robot. Ubuntu estaba bloqueado;
la nueva version del panel aun NO se instalo en la VM. El editor anterior de
limites si habia quedado instalado y abierto en la sesion previa.

## Instalación del panel tras desbloqueo

El usuario desbloqueó Ubuntu. Se ejecutó la actualización panel.py y se registró
el acceso ABB panel didactico. Se abrió /home/jose/abb-real/abrir_panel.sh con ROS
Humble, nodo de vista previa y RViz en dominio 45. Se observaron seis deslizadores,
pestañas XYZ y velocidad, botones de preparación/reproducción y envío físico
bloqueado. RViz mostró Global Status OK. Durante el uso, J3 cambió de 30° a
-90,399° y la postura y XYZ del modelo se actualizaron (X=-330,044 mm,
Y=0,000 mm, Z=818,562 mm). No hubo órdenes físicas ni conexión EGM.

Los ajustes posteriores de distribución (pestañas con desplazamiento y posición
inicial de ventanas) quedaron en la fuente y el ZIP, probados offscreen; no se
reinició el panel en uso para aplicarlos durante esta sesión.

## Cambio solicitado: real naranja y destino blanco

Preparada panel_sombra.py con escena dual real/goal, material blanco con alpha
0,45, naranja exclusivo de lectura real, sin animaciones en la interfaz. Una
ventana separada guarda un único porcentaje de velocidad para todos los ejes,
como parámetro destinado al movimiento físico todavía no integrado. Ejecutar
sigue deshabilitado. Se verificó con dobles que mover el destino no modifica
los ángulos reales publicados, que una lectura vieja se marca congelada, que
XYZ actualiza el destino y que la configuración tiene un solo control de velocidad.
Se validaron descripciones sin world duplicado y prefijos/topic distintos.
La instalación de esta versión en Ubuntu sigue pendiente: la pantalla volvió a
bloquearse y se solicitó al usuario que la desbloquee. La VM conserva el panel
anterior instalado; no confundirlo con la nueva interfaz del ZIP.

## Instalación confirmada de la escena doble

Tras el desbloqueo se ejecutó sombra.py, se cerró el panel anterior (PID 1768)
y se abrió /home/jose/abb-real/abrir_panel.sh, que ahora ejecuta panel_sombra.py.
Se observaron en Ubuntu los controles Angulos del destino / Destino XYZ, el
botón de velocidad única aparte y la sombra blanca semitransparente en RViz.
No hay animación de trayectorias. El panel indica SIN LECTURA REAL y no muestra
un naranja de muestra: el usuario puede conectar la lectura del ABB desde el
botón correspondiente. No se enviaron consignas físicas. El botón Ejecutar
permanece deshabilitado porque el programa RAPID/EGM aún no fue identificado.
