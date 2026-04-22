import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class UartBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.serial_port = serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=0.1)
        self.get_logger().info('Mostek UART uruchomiony. Czekam na /cmd_vel...')

    def cmd_vel_callback(self, msg):
        v_lin = round(msg.linear.x, 3)
        v_ang = round(msg.angular.z, 3)
        
        frame = f"{v_lin},{v_ang}\n"
        
        self.get_logger().info(f"Wysylam po UART: {frame.strip()}")
        
        self.serial_port.write(frame.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
