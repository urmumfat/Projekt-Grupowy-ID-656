import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import signal
import sys
import os

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.x_data = []
        self.y_data = []
        self.get_logger().info('Rejestrator trasy uruchomiony. Zbieram dane...')

    def odom_callback(self, msg):
        # Zapisujemy pozycję x i y
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def save_plot(self):
        if not self.x_data:
            self.get_logger().warn('Brak danych do wyrysowania trasy.')
            return

        self.get_logger().info(f'Generowanie wykresu z {len(self.x_data)} punktów...')
        
        plt.figure(figsize=(10, 10))
        plt.plot(self.x_data, self.y_data, label='Trasa robota', color='blue', linewidth=2)
        plt.scatter(self.x_data[0], self.y_data[0], color='green', label='Start', zorder=5)
        plt.scatter(self.x_data[-1], self.y_data[-1], color='red', label='Koniec', zorder=5)
        
        plt.title('Przemieszczenie robota (Odometria rf2o)')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') # Zachowanie skali 1:1
        
        # Zapis do pliku w folderze projektu
        output_path = '/my_robot_code/my_robot_code/trasa_robota.png'
        plt.savefig(output_path)
        self.get_logger().info(f'Wykres zapisany pomyślnie: {output_path}')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()

    def signal_handler(sig, frame):
        node.save_plot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Przechwytywanie sygnału SIGTERM wysyłanego przez menedżera
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

if __name__ == '__main__':
    main()
