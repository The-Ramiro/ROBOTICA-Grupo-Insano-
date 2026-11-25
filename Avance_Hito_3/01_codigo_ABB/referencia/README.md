# Proyecto ABB IRB 120 · IMT-342

Modelo cinemático, simulación ROS 2 y preparación de la integración con el brazo ABB IRB120-3/0.6. La prioridad actual es completar el inventario del controlador y obtener lectura real de estado antes de preparar movimiento. Actualización documental: **20 de octubre de 2026**.

## Empezar aquí

| Necesito… | Abrir |
|---|---|
| Saber qué conocemos del brazo y cómo conectarlo | [Conexión ABB: decisión, requisitos y procedimiento](docs/conexion_abb/README.md) |
| Ver las fotos y qué demuestra cada una | [Catálogo de evidencia fotográfica](fotos/README.md) |
| Preparar la semana 12 y los Hitos 3/4 integrados | [Estado y plan Hitos 3/4](docs/hitos_3_4/README.md) |
| Trabajar en la entrega del Hito 2 | [Paquete y notebook vigentes](hito2/README.md) · [Informe IEEE](docs/hito2/informe_ieee/README.md) |
| Abrir el material del docente | [Guías, plantillas y rúbricas](<archivos que hay que presentar/README.md>) |
| Encontrar las entregas del equipo | [Índice de entregables](entregables/README.md) |
| Usar la simulación del Hito 1 | [Instrucciones ROS](docs/hito1/README.md) |
| Entender dónde se trasladaron los archivos | [Mapa y registro de organización](docs/organizacion/README.md) |

## Estado comprobado y pendientes

| Elemento | Estado |
|---|---|
| Manipulador | IRB120-3/0.6, serie 120-101516; placa fotografiada |
| Opciones del controlador | **689-1 EGM, 616-1 PC Interface y 623-1 Multitasking** visibles en FlexPendant |
| Red de servicio | **192.168.125.1/24**, sin gateway; pantalla fotografiada |
| Puerto | Etiqueta **X2 Service 10/100** legible; no se realizó conexión desde Linux |
| RobotWare completo | **Pendiente**: las fotos todavía no muestran su versión |
| Controlador exacto y revisión | Pendientes de placa; IRC5 Compact es identificación visual provisional |
| Integración física | **No verificada**; diagnóstico de lectura preparado y probado offline |
| Driver ROS 2 | `PickNikRobotics/abb_ros2`, candidato condicionado a versión y adaptación al IRB 120 |
| Entorno Linux | Ubuntu 26.04.1 y Jazzy observados; compatibilidad del driver por comprobar |

La guía docente distingue G0 inventario, G1 lectura sin movimiento, G2 validación FK/IK/J, G3 movimiento con GO del laboratorio y G4 tarea repetible. El estado `Manual / Guard Stop / Stopped` corresponde al momento de las fotos. La captura de Backup and Restore muestra el menú, no un respaldo realizado.

## Carpetas

```text
proyecto/
├── README.md
├── model_manifest.yaml            # Versiones, modelos, evidencias y pendientes
├── hito2/                         # Código, notebook, figuras y resultados actuales
├── ros2_ws/                       # Paquetes ROS existentes
├── scripts/                       # Diagnóstico ABB y utilidades
├── docs/
│   ├── conexion_abb/              # Conexión, inventario y evidencias
│   ├── hito1/                     # Modelo y uso de la simulación
│   ├── hito2/                     # Informe IEEE y revisiones
│   ├── hitos_3_4/                 # Plan ECI-3/4
│   └── organizacion/              # Traslados y verificaciones
├── fotos/
│   ├── equipo/                    # Robot, placas y gabinete
│   ├── flexpendant/               # Opciones y pantallas
│   └── conexiones/                # Puertos y cableado
├── archivos que hay que presentar/
│   ├── hito_2/                    # Documentos originales del docente
│   └── actualizacion_semana_12/   # Guías y rúbrica de Hitos 3/4
├── entregables/                   # Índice del equipo y finales de Hito 1
├── referencias/                   # Bibliografía y repositorios
└── archivo/                       # Versiones y generación histórica
```

## Modelos y unidades

Hito 2 usa TCP a **0.150 m** de `tool0`; el modelo ROS anterior usa **0.100 m**. El TCP físico sigue pendiente. Hay que alinear base, herramienta, orden, signos y offsets articulares antes de comparar contra el controlador. [Manifiesto](model_manifest.yaml).

El acuerdo entre DH y un URDF derivado del mismo DH comprueba implementaciones de la cadena, pero no aporta por sí mismo una fuente geométrica independiente del robot ni validación física ABB. Para el Hito 2, la verificación geométrica independiente de la FK se hace contra el **modelo oficial de ROS-Industrial** (`abb_irb120_support`): coinciden en pose completa hasta 1e-15 (ver `hito2/irb120/rosi_model.py`).

## Primera herramienta disponible

Desde la raíz, este comando guarda únicamente información de la PC:

```bash
python3 scripts/abb_lectura_rws.py --output docs/conexion_abb/evidencias/inventario_pc.json
```

La futura prueba contra el controlador se describe en la [guía de conexión](docs/conexion_abb/README.md), después de verificar RobotWare, enlace Ethernet y acceso supervisado. El script no arranca RAPID, motores ni EGM. Sus ocho pruebas offline están registradas; no equivalen a una prueba sobre el brazo.

Los [documentos del docente](<archivos que hay que presentar/README.md>) son referencias. Las [versiones anteriores](archivo/README.md) son antecedentes. Los entornos virtuales y directorios de compilación permanecen en sus ubicaciones y están excluidos de Git.
