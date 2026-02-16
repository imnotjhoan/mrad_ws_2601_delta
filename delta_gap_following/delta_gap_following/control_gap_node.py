import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math


class GapFollowControl(Node):

    def __init__(self):
        super().__init__('gap_follow_control')

        # -- PARAMETERS --
        self.declare_parameter('kp', 1.0)                    # Proportional gain
        self.declare_parameter('kd', 0.5)                    # Derivative gain
        self.declare_parameter('max_steering', 4.0)          # Max steering angle (radians)
        self.declare_parameter('min_steering', -4.0)         # Min steering angle (radians)
        self.declare_parameter('forward_velocity', 5.0)      # Constant forward velocity (m/s)
        self.declare_parameter('kp_vel', 0.1)                # Velocity reduction gain

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.min_steering = float(self.get_parameter('min_steering').value)
        self.kp_vel = self.get_parameter('kp_vel').value


        # State variables for derivative calculation
        self.prev_angle_error = 0.0
        self.prev_time = None

        # Subscription to gap_angle topic
        self.gap_sub = self.create_subscription(
            Twist,
            '/cmd_ang_tcc',
            self.gap_callback,
            10
        )

        # Publisher for control commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_key',
            10
        )

        self.get_logger().info('Gap Follow Control node initialized')

    def gap_callback(self, msg: Twist):
        
        current_time = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_angle_error = 0.0
            return

        # Calculate time step
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Convert angle from degrees to radians
        gap_angle_deg = msg.angular.z
        gap_angle_rad = math.radians(gap_angle_deg)
        
        # The error is the angle we need to turn (positive = turn left, negative = turn right)
        angle_error = gap_angle_rad

        # Calculate error rate (derivative)
        if dt > 0:
            angle_error_rate = (angle_error - self.prev_angle_error) / dt
        else:
            angle_error_rate = 0.0

        self.prev_angle_error = angle_error

        # PD Control Law: steering = Kp*error + Kd*error_rate
        # steering_angle = self.kp * angle_error + self.kd * angle_error_rate

        steering_angle = self.kp * angle_error
        # # Apply saturation (min and max limits)
        # steering_angle = max(
        #     self.min_steering,
        #     min(self.max_steering, steering_angle)
        # )

        # Adjust velocity based on steering angle magnitude
        # Reduce speed when turning sharply

        # Create and publish command
        cmd = TwistStamped()
        cmd.header.stamp = current_time.to_msg()
        cmd.twist.linear.x = self.forward_vel
        cmd.twist.angular.z = steering_angle

        self.cmd_pub.publish(cmd)
        


        # Logging
        self.get_logger().info(
            f"Gap angle={gap_angle_deg:.1f}° | "
            f"Error={math.degrees(angle_error):.1f}° | "
            f"Error rate={math.degrees(angle_error_rate):.1f}°/s | "
            f"Steering={math.degrees(steering_angle):.1f}° | "
            f"Velocity={self.forward_vel:.2f}m/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()