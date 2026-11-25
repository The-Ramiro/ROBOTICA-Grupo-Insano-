# Control automático y versión manual

La versión manual completa quedó guardada en
`IMT342/versiones/ABB_manual_2026-11-18`, junto con un manifiesto SHA-256.
En Ubuntu, el instalador conserva `~/abb-manual-2026-11-18` antes de instalar
el panel automático.

## Abrir el panel

`bash ~/abb-real/abrir_panel.sh` abre el modo automático.
`bash ~/abb-real/abrir_automatico.sh` abre el mismo panel.
`bash ~/abb-real/abrir_manual.sh` abre el panel manual con `ServidorDirecto`.
La copia completa de la versión manual conserva su propio `abrir_panel.sh`.

## Preparación del controlador

En **Conectar**, indicar IP, usuario, contraseña y el archivo
`rapid/IMT342_Automatico.mod`. Sus variables `autoInicio`,
`autoDestino`, `autoOrden` y `autoSesion` son distintas de las del módulo manual.
El encargado selecciona AUTO con la llave y confirma localmente el cambio.
La zona y las protecciones de la celda deben estar preparadas para automático.
Se requieren los permisos UAS de escritura, habilitación de motores y arranque
remoto mediante RWS. La PC usa mastership del dominio RAPID.

Al aceptar **OK**, la PC detiene RAPID en curso, verifica identidad y versión de
RobotWare, sube y carga el módulo compatible si hace falta y comprueba la
compilación. Guarda una copia local si reemplaza una versión distinta del
módulo propio. Comprueba la postura y la altura, resuelve las órdenes detenidas,
coloca el puntero en `IMT342_Automatico/ServidorAutomatico`, solicita Motors On
y arranca la recepción RAPID. La sombra se sincroniza con la postura real.
El diálogo acepta el módulo compatible con este protocolo del panel.

El arranque de la rutina descarta los valores de destino anteriores y queda
esperando una secuencia nueva. Editar ángulos, XYZ, roll/pitch/yaw o arrastrar
la esfera y sus anillos actualiza la vista previa. **Ejecutar** envía el destino
revisado. Cada orden incluye
la postura inicial, lectura de vuelta del destino y una secuencia única.
El panel espera confirmación de llegada antes de aceptar otra orden.
La IK recibe la posición y la orientación indicadas.

## Si el controlador no entrega las articulaciones

El error ABB `SYS_CTRL_E_MECSTA_NOT_READY` impide preparar la sesión.
Si el registro muestra `20032` (contadores de vueltas sin actualizar) o
`50057` (articulación sin sincronizar), el encargado debe resolver la
sincronización siguiendo el manual del robot. El panel no actualiza
contadores ni fuerza lecturas para habilitar movimientos.

## Trayectorias por puntos

En **Trayectorias**, colocar la sombra desde Control y pulsar **Añadir sombra
como punto**. La tabla contiene hasta 100 posturas, con XYZ en milímetros;
al seleccionar una fila aparecen los seis ángulos en grados. Se puede mostrar
el punto en la sombra, reemplazarlo por la postura editada, reordenar, eliminar,
vaciar, guardar y abrir archivos JSON. Abrir un archivo no conecta ni ejecuta.

La línea celeste dibuja una estimación por interpolación articular y FK, con
P1…Pn. El acceso a P1 se dibuja desde la lectura actual del ABB; sin lectura
solo se muestra P1→P2→…→Pn, sin postura inicial supuesta. **Previsualizar sombra** anima únicamente el modelo blanco.
La animación es ilustrativa: no es una reproducción de tiempos ni de la
interpolación exacta de RobotWare. No detecta obstáculos físicos.

La velocidad global permanece visible debajo de todas las pestañas. Seleccionar
un porcentaje positivo, pulsar **Aplicar** y después **Ejecutar trayectoria**.
La PC vuelve a validar todos los tramos desde la postura real reciente y la
altura real de la brida, sube `autoRuta` completa y `autoCantidad`, comprueba
la lectura de vuelta y publica `autoOrden` una sola vez. Se mantienen los
rangos, la altura mínima de los extremos y la coherencia entre ABB y modelo.
Por indicación del operador, la cota conservadora de altura del tramo es
informativa y no bloquea el envío. El panel muestra la altura mínima muestreada
del recorrido ilustrativo; no es una garantía del recorrido real de ABB.

RAPID copia los puntos a una lista local, revisa rangos y alturas de sus extremos,
y realiza un MoveAbsJ con `vmax` y `fine` por punto. **Hay parada en cada punto**;
esta versión no ejecuta MoveL ni empalmes continuos. `autoTramo` indica el tramo
actual y `autoHecho` confirma toda la secuencia. Un movimiento individual escribe
`autoCantidad=0`, evitando reutilizar una lista anterior. El inicio coincidente
con P1 se omite; otros puntos consecutivos iguales se rechazan.

Durante la ejecución se bloquean edición, conexión y ajuste de velocidad en el
panel; **Detener** permanece disponible. Si la PC detecta un cambio externo del
porcentaje global, solicita Stop. La pérdida de Ethernet no garantiza detener
la lista que ya está en el controlador: usar las paradas físicas de la celda.

## Cálculos del panel

En **Control**, **Velocidad del brazo real** permite elegir un único porcentaje
global entre 0 y 100 y enviarlo con **Aplicar**. La lectura confirmada viene
de `/rw/panel/speedratio`. Los cambios se aplican entre movimientos y requieren
AUTO. Si se edita un porcentaje, **Ejecutar** espera a que se aplique y confirme.
El 100% corresponde a `vmax`, la velocidad máxima configurada por ABB para el
movimiento y el robot. No garantiza que cada eje alcance su máximo: intervienen
la sincronización, la aceleración, la carga, la postura y las restricciones ABB.
Cada conexión restablece y confirma 0% antes de iniciar el receptor RAPID;
se debe elegir un porcentaje y pulsar Aplicar antes de Ejecutar.
Los límites de análisis LSPB usan sus propios parámetros.
Cada ajuste real queda registrado en `~/abb-real/ordenes/velocidad_*.json`.
Un porcentaje de 0% impide publicar un movimiento nuevo. La espera de llegada
se amplía para porcentajes pequeños.

**Control** mantiene sincronizados los seis ángulos, XYZ y roll/pitch/yaw. La
convención es R = Rz(yaw) Ry(pitch) Rx(roll), en grados. **Mover** muestra
flechas en los ejes de la base. **Girar** muestra anillos en los ejes locales
de la brida. **Ambos** reúne los controles. El clic derecho en la esfera abre
ajustes finos por pasos configurables, inicialmente 0,1 mm y 0,1°.
Una pose inalcanzable regresa a la última postura válida al soltar el control.
Los análisis pesados se posponen hasta terminar el arrastre y los cálculos
obsoletos se cancelan al cambiar el destino.

**Verificación** muestra los errores articulares, de posición y orientación
frente al ABB, los residuales de IK y las alternativas de la misma pose.
Los errores de orientación ABB requieren los cuaterniones RWS recientes.

**Cinemática** muestra T(base→brida), el Jacobiano geométrico, sus seis valores
singulares, el número de condición, rango numérico y manipulabilidad. Para la
SVD se escalan las filas angulares por 0,2 m.

**LSPB** muestra posición, velocidad y aceleración sincronizadas de los seis
ejes. La duración se amplía para respetar el porcentaje de velocidad ABB y la
aceleración de diseño seleccionados. Las ecuaciones siguen el material
[hito2/irb120/lspb.py](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/hito2/irb120/lspb.py).
Es un perfil de análisis. El ABB ejecuta MoveAbsJ con su planificador interno.

**Dinámica** estima M(q), c(q,q̇), g(q) y τ = Mq̈+c+g usando las masas,
centros de masa e inercias del URDF. El deslizador de tiempo muestra las
contribuciones de par, energías cinética/potencial, potencia y rapidez de la
brida. Es un modelo sin fricción, reductores ni carga adicional. Las inercias
requieren validación física, especialmente la aproximación de link_6. Los
límites de par del fabricante quedan pendientes de datos.

La dinámica [1R de clase](https://github.com/Marcelo767n/robotica-IMT342/blob/194c0778d51754fc20f492d1a7d67c30d7f5c9be/2-2026/IMT342/equipo/proyecto/hito2/explicacion/07_E_dinamica_1R.md)
sirve como referencia docente. El panel calcula la estimación de seis ejes del
URDF. La revisión del repositorio usó la rama proyectoHito3y4 en el commit
194c0778d51754fc20f492d1a7d67c30d7f5c9be.

## Paradas y recuperación

**Detener** deshabilita el envío y solicita Stop de RAPID por RWS.
La PC comprueba periódicamente el modo, los motores, RAPID, el puntero,
la lectura reciente y el número de sesión. Una interrupción deshabilita
la sesión. La recuperación se hace mediante **Conectar**.
Una publicación con respuesta perdida queda sin reintento.

Al detener una orden pendiente, la PC confirma RAPID detenido y postura estable,
termina el contexto de la rutina propia y marca esa secuencia como cancelada
(estado -1). La confirmación de cancelación se escribe al final, con lectura de
vuelta. La sesión siguiente prepara una orden nueva desde la postura medida.

Una parada por software requiere comunicación. Si se pierde Ethernet durante
`MoveAbsJ`, el controlador puede continuar hasta el destino ya recibido.
Las paradas físicas y las protecciones de la celda proporcionan la detención
independiente de la PC. El indicador del panel informa si Stop quedó sin confirmar.

Los límites y las alturas de los extremos usan el modelo del proyecto.
La cota conservadora de altura intermedia no bloquea el movimiento (ajuste
solicitado por el operador el 19/11/2026).
El entorno físico, los obstáculos y las colisiones de eslabones requieren
la comprobación de la celda. El RAPID usa `vmax` respetando los límites ABB.
El porcentaje global se configura en el controlador; 0% bloquea órdenes nuevas.

## Estado de validación

El protocolo se probó con RWS simulado, incluyendo modo manual, Guard Stop,
Emergencia, módulo distinto, orden anterior pendiente, lectura de vuelta,
reinicio de RAPID y pérdida de respuesta al publicar. La compilación del
módulo con vmax se verificó en RobotWare 6.12 el 19/11/2026: carga y
compilación sin errores, RAPID detenido, porcentaje 0% y sin cambio de postura.
No se ejecutó una prueba física de velocidad máxima.
Abrir el panel deja la recepción automática deshabilitada. Aceptar **Conectar**
prepara e inicia la recepción. **Ejecutar** publica el movimiento.

## Documentación de ABB

- [Mastership RAPID](https://developercenter.robotstudio.com/api/rwsApi/msh_domain_request_page.html)
- [Puntero a rutina](https://developercenter.robotstudio.com/api/rwsApi/rapid_task_pcp_set_to_routine.html)
- [Motors On](https://developercenter.robotstudio.com/api/rwsApi/panel_ctrlstate_set_page.html)
- [Arranque RAPID](https://developercenter.robotstudio.com/api/rwsApi/rapid_execution_start_page.html)

Trayectorias (19/11/2026): 58 pruebas de software y prueba visual de la línea
y puntos en RViz/Humble aprobadas. El ABB no respondió en 192.168.125.1 durante
esta revisión; la compilación de la ampliación y la ejecución física de la
lista de puntos quedan pendientes. No se enviaron movimientos físicos.

## Panel y análisis separados (20/11/2026)

La pantalla principal contiene solo Control y Trayectorias. Análisis abre una
ventana independiente con gráficas grandes y desplazamiento vertical y horizontal.
Las páginas originales de verificación, Jacobiano, LSPB y dinámica siguen en
Postura y fórmulas dentro de esa ventana, con sus datos y ajustes.

Calcular recorrido completo genera perfiles LSPB de todos los tramos, fases de
aceleración/velocidad constante/desaceleración y marcas de llegada P1…Pn.
El acceso a P1 solo se incluye si hay una lectura real reciente del ABB.
Se muestran q, q̇, q̈, posición y velocidad de la brida, σmín, log10 de condición,
manipulabilidad y dinámica URDF muestreada (par, energía y potencia). En Enlaces
XYZ se selecciona el origen de cada enlace; sus derivadas son numéricas.
La tabla ubica los avisos por punto y tiempo previsto. El umbral σmín inicial
0,001 corresponde al Jacobiano con filas angulares escaladas por 0,2 m;
es informativo, no un límite de seguridad ni una causa automática de parada.

Estos tiempos y perfiles son estimaciones. ABB ejecuta MoveAbsJ con su propio
planificador: no se afirma que reproduzca exactamente el LSPB dibujado.
Los análisis largos se calculan en segundo plano y admiten cancelación.

ABB medido y sucesos registra los ángulos leídos, derivadas numéricas con tiempos
reales de recepción y diferencias entre FK y el TCP comunicado por ABB.
No se fabrican lecturas ausentes; derivadas sin muestras suficientes y huecos
de red se muestran sin datos. Un cursor permite leer valores en las gráficas.
Se muestran mensajes de la aplicación y pueden abrirse registros RWS de órdenes
fallidas. El último punto observado puede ser aproximado por el muestreo.
No se atribuye una parada a una singularidad solo porque coincidan en el tiempo.
Los registros de lecturas y mensajes se guardan al cerrar en ordenes/lecturas_*.json,
y también se pueden guardar/abrir desde la ventana de análisis.

Se admiten 100 puntos en la lista. El receptor RAPID conserva su búfer de 20:
la PC valida todos los tramos antes del primer envío y manda grupos confirmados
manteniendo un único porcentaje y la edición bloqueada durante la secuencia.
No se reintenta un envío de respuesta incierta. Se conservan las paradas fine.
La ampliación a 100 no necesita modificar el módulo ya preparado de 20 puntos.

La entrada de la bolita se deshabilita durante la animación de la sombra,
conservando estables sus controles gráficos en RViz/Humble. No se vacían ni
reinsertan al iniciar o finalizar, para evitar un fallo gráfico observado.
La pose completa sigue a la brida durante la reproducción y al terminar. Se eliminó la doble
actualización de orientación del manipulador por ciclo. Abrir, calcular,
previsualizar o revisar registros no envía movimientos al controlador.

Validación 20/11/2026: 67 pruebas de software, interfaz Qt de ejecución/Stop,
reproducción completa en RViz/Humble sin cierre y manipulador en la pose de la
brida. Informe de 100 puntos y seis enlaces: 6535 muestras, probado en segundo
plano. Sin prueba de movimiento físico de las listas de 100 puntos.

## Análisis simplificado (20/11/2026)

Gráficas de trayectoria contiene únicamente posición, velocidad y aceleración
articulares LSPB. Se retiraron de esa página las gráficas de índices del
Jacobiano, movimiento de brida, par, energía y potencia. ABB medido y sucesos
y Postura y fórmulas se mantienen. Se retiró Enlaces XYZ y su entrada del menú.

Singularidades muestra tres medidas geométricas del URDF: ángulo agudo entre
los ejes J4/J6, desalineación brazo-antebrazo (hombro/codo/centro de muñeca)
y distancia del centro de muñeca al eje J1. Cero indica alineación. Se usan
bandas orientativas de 5 grados y 10 mm; no bloquean movimientos ni afirman
que haya contacto entre las mallas. Los ceros geométricos se contrastaron
con la pérdida de rango del Jacobiano en las pruebas del modelo.

La tabla muestra el tipo, primer instante de aviso y mínimo por tramo. Incluye
los extremos compartidos por tramos. El deslizador permite elegir un instante
y Ver postura en sombra cambia solo el modelo blanco, con el envío físico
explícito separado. Los cálculos usan el recorrido LSPB estimado, no el perfil
interno exacto del ABB. Los Jacobianos detallados siguen en Postura y fórmulas.

## Estado estable de Ejecutar (20/11/2026)

El panel principal calcula una sola vez por ciclo si Ejecutar está disponible.
Las capas de vista previa y trayectorias no vuelven a cambiar ese mismo botón.
Las consultas GET periódicas de estado y velocidad no deshabilitan una orden
si siguen existiendo lecturas recientes confirmadas. Aplicar un cambio de
velocidad, falta de conexión o lecturas recientes, preparación/parada, movimiento,
edición pendiente y vista previa continúan bloqueando el envío. El mismo criterio
se comprueba al hacer clic y el motivo queda en la ayuda del botón.

Se verificó en Qt que no hay cambios de habilitación durante lecturas pendientes,
tanto en movimiento individual como en trayectorias, y que sin conexión permanece
inhabilitado. Las validaciones RWS/RAPID al publicar las órdenes se conservan.
