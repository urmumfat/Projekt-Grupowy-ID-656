import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal

class SystemManager(Node):
    def __init__(self):
        super().__init__('system_manager')
        self.srv = self.create_service(SetBool, 'manage_slam', self.manage_slam_callback)
        self.launch_process = None
        self.get_logger().info('Menedżer systemu gotowy.')

    def manage_slam_callback(self, request, response):
        if request.data:  # Włączanie
            if self.launch_process is None or self.launch_process.poll() is not None:
                self.get_logger().info('Uruchamianie pliku Launch (Lidar + rf2o + SLAM)...')
                
                # TUTAJ WPISZ NAZWĘ SWOJEGO PLIKU LAUNCH
                self.launch_process = subprocess.Popen(
 			   "source /ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch /my_robot_code/my_robot_code/slam.py",
   			    shell=True,
    	                    executable="/bin/bash",
    		            preexec_fn=os.setsid
			)
                
                response.success = True
                response.message = "System SLAM włączony"
            else:
                response.success = False
                response.message = "System już działa"
        else:  # Wyłączanie
            if self.launch_process is not None and self.launch_process.poll() is None:
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait()
                self.launch_process = None
                response.success = True
                response.message = "System SLAM wyłączony"
            else:
                response.success = False
                response.message = "System nie był uruchomiony"
        
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    manager = SystemManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info('Zamykanie menedżera (Ctrl+C)...')
    finally:
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
