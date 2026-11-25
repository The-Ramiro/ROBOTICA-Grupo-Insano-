"""Escena RViz del ABB real, su destino y el marco fijo del laboratorio."""
import copy
import xml.etree.ElementTree as ET


def robot_xml(text, *, white=False):
    root = ET.fromstring(text)
    for joint in list(root.findall('joint')):
        if joint.find('child').get('link') == 'base_link':
            root.remove(joint)
    for link in list(root.findall('link')):
        if link.get('name') == 'world':
            root.remove(link)
    if white:
        for visual in root.findall('.//visual'):
            material = visual.find('material')
            if material is not None:
                visual.remove(material)
            material = ET.SubElement(visual, 'material', {'name': 'destino_blanco'})
            ET.SubElement(material, 'color', {'rgba': '1 1 1 1'})
    return ET.tostring(root, encoding='unicode')


SUPPORT_TOPIC = '/imt342/soporte'


def support_marker(mesh):
    from visualization_msgs.msg import Marker, MarkerArray

    marker = Marker()
    marker.header.frame_id = 'world'
    marker.ns = 'soporte_abb'
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = mesh.as_uri()
    marker.scale.x = marker.scale.y = marker.scale.z = .001  # STL en mm.
    # El interior del marco en +X queda a 0,505 m del eje J1.
    marker.pose.position.x = .505 - .7505
    marker.pose.position.y = .8005 / 2.
    marker.pose.position.z = .20
    # Gira 180° alrededor de X para que las patas queden debajo del marco.
    marker.pose.orientation.x, marker.pose.orientation.w = 1., 0.
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = .12, .43, .72, .90
    return MarkerArray(markers=[marker])


def rviz_config(base):
    config = copy.deepcopy(base)
    displays = config['Visualization Manager']['Displays']
    template = next(d for d in displays if d['Class']=='rviz_default_plugins/RobotModel')
    replacements = []
    for prefix, name, alpha in [('real', 'NARANJA · posicion real ABB', 1.),
                                 ('goal', 'BLANCO · destino propuesto', .45)]:
        robot = copy.deepcopy(template)
        robot.update({'Name': name, 'TF Prefix': prefix, 'Alpha': alpha})
        robot['Description Topic']['Value'] = f'/{prefix}/robot_description'
        replacements.append(robot)
    manager = config['Visualization Manager']
    manager['Displays'] = [d for d in displays if d['Class']=='rviz_default_plugins/Grid']+replacements
    manager['Displays'].append({'Class': 'rviz_default_plugins/InteractiveMarkers',
                                'Name': 'Brida objetivo', 'Enabled': True,
                                'Interactive Markers Namespace': '/imt342/objetivo',
                                'Show Axes': False, 'Show Descriptions': False})
    manager['Displays'].append({'Class': 'rviz_default_plugins/MarkerArray',
                                'Name': 'Trayectoria de la sombra', 'Enabled': True,
                                'Topic': {'Value': '/imt342/trayectoria',
                                                 'Reliability Policy': 'Reliable',
                                                 'Durability Policy': 'Volatile',
                                                 'History Policy': 'Keep Last', 'Depth': 1},
                                'Queue Size': 10})
    manager['Displays'].append({'Class': 'rviz_default_plugins/MarkerArray',
                                'Name': 'Marco del ABB', 'Enabled': True,
                                'Topic': {'Value': SUPPORT_TOPIC, 'Reliability Policy': 'Reliable',
                                          'Durability Policy': 'Transient Local',
                                          'History Policy': 'Keep Last', 'Depth': 1},
                                'Queue Size': 1})
    manager['Tools'].append({'Class': 'rviz_default_plugins/PublishPoint',
                             'Topic': '/imt342/punto', 'Single click': True})
    config['Panels'] = []
    config['Window Geometry'].update({'Width': 550, 'Height': 700, 'X': 710, 'Y': 40,
                                      'Hide Left Dock': True, 'Hide Right Dock': True})
    return config
