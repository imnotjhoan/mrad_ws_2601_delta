import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class EmergencyBrake(Node):

    def __init__(self):
        super().__init__('emergency_brake')

        # Suscripción al tiempo de colisión
        self.create_subscription(
            Float32,
            '/tcc',
            self.tcc_callback,
            10
        )
        
        # suscripcion al joystick del control
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_joy',
            self.joy_callback,
            10
        )

        # Publicador de comandos de velocidad (con prioridad)
        self.cmd_pub_key = self.create_publisher(
            TwistStamped,
            '/cmd_vel_key',
            10
        )

        self.tcc_threshold = 1.3  # segundo en el que se activa el freno de emergencia
        self.timer = None   
        self.counter = 0
        self.joy_linear_x = 0.0
        self.get_logger().info('Emergency brake initialized')
        self.velocity = 0.0  # m/s, velocidad de retroceso durante el frenado de emergencia

    def joy_callback(self, msg):
        
        self.joy_linear_x = msg.twist.linear.x
        
    def tcc_callback(self, msg):
        tcc = msg.data

        if tcc < self.tcc_threshold:
            if self.timer is None:  # iniciar temporizador
                self.counter = 0
                self.velocity = -50.0  # velocidad inicial de retroceso
                self.timer = self.create_timer(0.1, self.brake_timer)  
                self.get_logger().warn(f'TCC={tcc:.2f}s → Activando freno')
                
        elif  abs(self.joy_linear_x) < 0.1:  # cancelar temporizador si el joystick no está activo
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
                self.get_logger().info('Freno desactivado')

    def brake_timer(self):
        twist_stamped = TwistStamped()
        
        twist_stamped.twist.linear.x = self.velocity
        twist_stamped.twist.angular.z = 0.0
        self.cmd_pub_key.publish(twist_stamped)
        self.get_logger().warn(f'Retrocediendo... ({self.counter+1}/5)')

        self.counter += 1
        if self.counter >= 5 and abs(self.joy_linear_x) < 0.1:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info('Freno completado')
            
            
        elif self.counter >= 5 and abs(self.joy_linear_x) >= 0.1:
            self.velocity = 0.0
            self.get_logger().warn('Joystick aún activo, freno no completado')



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
