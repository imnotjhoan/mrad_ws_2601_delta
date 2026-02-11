import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

class tccNode(Node):

    def __init__(self):
        super().__init__('tcc_node')

        # Parámetros
        self.declare_parameter('theta_roi_deg', 2.0)
        self.theta_roi = math.radians(
            self.get_parameter('theta_roi_deg').value
        )

        self.vx = 0.0

        # Subscriptores
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/diffdrive_controller/odom',
            self.odom_callback,
            10
        )

        # Publicador
        self.tcc_pub = self.create_publisher(
            Float32,
            '/tcc',
            10
        )

        self.get_logger().info('tcc node initialized')

    def odom_callback(self, msg):
        self.vx = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        if self.vx <= 0.01:
            return

        tcc_min = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment

            if abs(theta) > self.theta_roi:
                continue

            r_dot = -self.vx * math.cos(theta)

            if r_dot >= 0.0:
                continue

            tcc_i = r / (-r_dot)

            if tcc_i < tcc_min:
                tcc_min = tcc_i

        if tcc_min < float('inf'):
            msg_out = Float32()
            msg_out.data = tcc_min
            self.tcc_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = tccNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
