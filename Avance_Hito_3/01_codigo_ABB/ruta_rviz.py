"""Dibujo persistente del recorrido estimado y sus puntos numerados."""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class RouteDrawing:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(MarkerArray, '/imt342/trayectoria', 1)
        self.latest = MarkerArray()

    def update(self, path, vertices):
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        def marker(index, kind):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'ruta', index, kind, Marker.ADD
            m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b, m.color.a = .15, .85, 1., 1.
            result.markers.append(m)
            return m
        if len(path):
            line = marker(0, Marker.LINE_STRIP)
            line.scale.x = .003
            line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in path]
        for index, p in enumerate(vertices):
            dot = marker(1+index*2, Marker.SPHERE)
            dot.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            dot.scale.x = dot.scale.y = dot.scale.z = .012
            label = marker(2+index*2, Marker.TEXT_VIEW_FACING)
            label.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]+.025))
            label.scale.z = .022
            label.text = f'P{index+1}'
        self.latest = result
        self.publish()

    def publish(self):
        self.publisher.publish(self.latest)
