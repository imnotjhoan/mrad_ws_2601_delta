import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

class GapDistanceNode(Node):
    def __init__(self):
        super().__init__('Gap_Distance_Node')
        
        # Parámetros configurables
        self.declare_parameter('min_distance', 0.3)  # Distancia mínima 
        self.declare_parameter('max_distance', 12.0)  # Distancia máxima
        self.declare_parameter('circle_radius', 0.2)  # Radio de "burbuja" alrededor de obstáculos cercanos
        self.declare_parameter('robot_radius', 0.1)  # Radio de "burbuja" alrededor de obstáculos cercanos
        
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.circle_radius = self.get_parameter('circle_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value        
        
        # Suscripción al LaserScan
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

        # Publicadores
        # Cambiado: ahora publica al tópico que lee el controlador
        self.cmd_ang_pub = self.create_publisher(Twist, '/cmd_ang_tcc', 10)
        self.marker_pub = self.create_publisher(Marker, '/gap_marker', 10)
        self.circles_marker_pub = self.create_publisher(Marker, '/safety_circles', 30) 
        
        self.get_logger().info("Nodo Follow the Gap Distance iniciado.")
        self.get_logger().info("Enviando ángulos objetivo al controlador en /cmd_ang_tcc")

    def _scan_callback(self, scan_msg):
        """Callback principal que implementa Follow the Gap"""
        self.last_scan = scan_msg

        # Obtener rangos desde el lidar
        ranges = list(scan_msg.ranges)
        N = len(ranges) #720 muestras
        
        if N == 0:
            return
        
        # Encontrar el punto más cercano (ignorando inf y nan)
        valid_ranges = [min(r, self.max_distance) if not math.isinf(r) else self.max_distance for r in scan_msg.ranges]

        if len(valid_ranges) == 0:
            # Todo es inf, ir adelante
            self._publish_target_angle(0.0)
            return
        
        # Crear circulos alrededor de TODOS los obstáculos
        processed_ranges = ranges.copy()
                
        # Encontrar el obstáculo más cercano válido
        closest_idx = None
        closest_dist = float('inf')

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < closest_dist:
                closest_dist = r
                closest_idx = i         
                
        if closest_idx is not None and closest_dist <= 3.0:
            circle_indices = self._get_circle_indices(
                closest_idx,
                N,
                scan_msg.angle_increment,
                closest_dist,
                self.robot_radius,
                self.circle_radius
            )

            for circle_idx in circle_indices:
                processed_ranges[circle_idx] = float('nan')

        
        self._publish_circle_markers(scan_msg, closest_idx, closest_dist)

        
        # Encontrar el gap más grande en el frente del robot
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        
        # Índices para parte  frontal [-90°, +90°]
        i_start = math.ceil(((-math.pi/2) - angle_min) / angle_inc)
        i_end = math.floor(((+math.pi/2) - angle_min) / angle_inc)
        
        i_start = max(0, min(N-1, i_start))
        i_end = max(0, min(N-1, i_end))
        
        forward_indices = list(range(i_start, i_end + 1))
        
        if len(forward_indices) == 0:
            self._publish_target_angle(0.0)
            return
        
        # Encontrar el gap más grande 
        largest_gap_start, largest_gap_end = self._find_best_gap(
            processed_ranges,
            forward_indices,
            angle_min,
            angle_inc
        )

        
        if largest_gap_start is None:
            self._publish_target_angle(0.0)
            self.get_logger().warn("No se encontró gap navegable.")
            return
        
        # Encontrar el mejor punto en el gap (el más profundo)
        best_idx = self._find_best_point_in_gap(
            processed_ranges, largest_gap_start, largest_gap_end
        )
        
        # Calcular el ángulo objetivo
        target_angle = angle_min + best_idx * angle_inc
        
        # Normalizar a [-pi, pi]
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi
        
        # Publicar ángulo objetivo al controlador
        self._publish_target_angle(target_angle)
        
        # Visualizar con marker
        self._publish_gap_marker(target_angle, scan_msg.header.frame_id)
        
        # Log
        gap_size = largest_gap_end - largest_gap_start + 1
        self.get_logger().info(
            f"Gap: tamaño={gap_size} | ángulo={math.degrees(target_angle):.1f}° | "
            f"distancia={processed_ranges[best_idx]:.2f}m"
        )

    def _get_circle_indices(self, center_idx, N, angle_inc, obstacle_dist, robot_radius_num, circle_radius_num):
        circle_indices = []
        
        total_radius = robot_radius_num + circle_radius_num
        
        # Calcular el ángulo que abarca el círculo desde el robot
        if obstacle_dist > 0 and obstacle_dist > total_radius:
            half_angle = math.asin(min(1.0, total_radius / obstacle_dist))
        elif obstacle_dist <= total_radius:
            half_angle = math.radians(90)
        else:
            half_angle = math.radians(30)
        
        circle_span = int(half_angle / angle_inc)
        
        min_span = int(math.radians(10) / angle_inc)
        circle_span = max(circle_span, min_span)
        
        for i in range(center_idx - circle_span, center_idx + circle_span + 1):
            if 0 <= i < N:
                circle_indices.append(i)
        
        return circle_indices

    def _score_gap(self, start_idx, end_idx, angle_min, angle_inc):
        gap_size = end_idx - start_idx + 1
        center_idx = (start_idx + end_idx) // 2
        center_angle = angle_min + center_idx * angle_inc

        # angle_penalty = abs(center_angle)
        angle_penalty = 0
        return gap_size / (1.0 + angle_penalty)

    def _find_best_gap(self, ranges, indices, angle_min, angle_inc):
        best_score = -1.0
        best_start = None
        best_end = None

        in_gap = False
        gap_start = None

        for i, idx in enumerate(indices):
            is_free = (not math.isnan(ranges[idx]) and ranges[idx] > self.min_distance)

            if is_free and not in_gap:
                in_gap = True
                gap_start = idx

            elif not is_free and in_gap:
                in_gap = False
                gap_end = indices[i - 1]
                score = self._score_gap(gap_start, gap_end, angle_min, angle_inc)

                if score > best_score:
                    best_score = score
                    best_start = gap_start
                    best_end = gap_end

        if in_gap:
            gap_end = indices[-1]
            score = self._score_gap(gap_start, gap_end, angle_min, angle_inc)
            if score > best_score:
                best_start = gap_start
                best_end = gap_end

        return best_start, best_end

    def _find_best_point_in_gap(self, ranges, start_idx, end_idx):
        """Encuentra el punto con mayor distancia en el gap"""
        if start_idx is None or end_idx is None:
            return 0
        
        best_idx = start_idx
        max_dist = ranges[start_idx]
        
        for idx in range(start_idx, end_idx + 1):
            if ranges[idx] > max_dist:
                max_dist = ranges[idx]
                best_idx = idx
        
        return best_idx

    def _publish_target_angle(self, target_angle):
        """Publica el ángulo objetivo al controlador"""
        cmd = Twist()
        cmd.angular.z = target_angle  # El controlador espera el ángulo en angular.z
        self.cmd_ang_pub.publish(cmd)

    def _publish_gap_marker(self, angle_rad, frame_id):
        """Publica un marker de visualización en RViz"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "gap_direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Convertir yaw a quaternion
        qz = math.sin(angle_rad / 2.0)
        qw = math.cos(angle_rad / 2.0)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        
        marker.scale.x = 1.5
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

    def _publish_circle_markers(self, scan_msg, closest_idx, closest_dist):
        # Si no hay obstáculo válido, no publicar nada
        if closest_idx is None or math.isinf(closest_dist) or math.isnan(closest_dist):
            return

        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "safety_circles"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.03

        marker.color.a = 0.9
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        total_radius = self.robot_radius + self.circle_radius
        num_segments = 32

        # Posición del obstáculo más cercano
        angle = scan_msg.angle_min + closest_idx * scan_msg.angle_increment
        cx = closest_dist * math.cos(angle)
        cy = closest_dist * math.sin(angle)

        for i in range(num_segments):
            a1 = 2.0 * math.pi * i / num_segments
            a2 = 2.0 * math.pi * (i + 1) / num_segments

            p1 = Point()
            p1.x = cx + total_radius * math.cos(a1)
            p1.y = cy + total_radius * math.sin(a1)
            p1.z = 0.0

            p2 = Point()
            p2.x = cx + total_radius * math.cos(a2)
            p2.y = cy + total_radius * math.sin(a2)
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

        self.circles_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GapDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
