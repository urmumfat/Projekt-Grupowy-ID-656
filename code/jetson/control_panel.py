import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
import subprocess

class ControlPanel(Node):
    def __init__(self):
        super().__init__('control_panel')
        
        # Klient do Twojego Menedżera SLAM
        self.slam_client = self.create_client(SetBool, '/manage_slam')
        
        # Klienci do wbudowanych serwisów w sllidar_ros2
        self.start_motor_client = self.create_client(Empty, '/start_motor')
        self.stop_motor_client = self.create_client(Empty, '/stop_motor')

    def call_motor(self, client, nazwa_akcji):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"Błąd: Serwis niedostępny! (Czy SLAM i Lidar są włączone?)")
            return
            
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        print(f"Polecenie wysłane: {nazwa_akcji}")

    def call_slam(self, state):
        if not self.slam_client.wait_for_service(timeout_sec=2.0):
            print("Błąd: Serwis SLAM niedostępny! (Czy menedżer na Jetsonie działa?)")
            return
            
        req = SetBool.Request()
        req.data = state
        future = self.slam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(f"Menedżer: {future.result().message}")

def main(args=None):
    rclpy.init(args=args)
    panel = ControlPanel()

    try:
        while rclpy.ok():
            print("\n" + "="*30)
            print("       PANEL STEROWANIA")
            print("="*30)
            print("1 - Włącz silnik lidaru")
            print("2 - Wyłącz silnik lidaru")
            print("3 - Włącz cały system SLAM")
            print("4 - Wyłącz cały system SLAM")
            print("5 - Sterowanie klawiaturą (Teleop)")
            print("0 - Wyjście")
            print("="*30)
            
            c = input("Wybierz akcję (0-5): ")
            
            if c == '1':
                panel.call_motor(panel.start_motor_client, "Włączono silnik")
            elif c == '2':
                panel.call_motor(panel.stop_motor_client, "Zatrzymano silnik")
            elif c == '3':
                panel.call_slam(True)
            elif c == '4':
                panel.call_slam(False)
            elif c == '5':
                print("\n>>> Uruchamiam klawiaturę... (Wciśnij Ctrl+C, aby wrócić do menu) <<<")
                subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            elif c == '0':
                print("Zamykanie...")
                break
            else:
                print("Nieznana opcja!")
    except KeyboardInterrupt:
        print("\nPrzerwano (Ctrl+C).")
    finally:
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
