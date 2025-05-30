import rclpy
from rclpy.node import Node
import rclpy.parameter
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
import serial
import time


w, h, x, y = 3,3,3,3
camera_size = [w,h]
center_position = [x, y]

class LaserControl(Node):
    def __init__(self):
        super().__init__('laser_control')

        #Parameter Declaration
        self.declare_parameter('manual_mode' , 'MANUAL_OFF')
        self.manual_mode = self.get_parameter('manual_mode').get_parameter_value().string_value

        self.allowed_modes = ['MANUAL_ON', 'MANUAL_OFF']

        if self.manual_mode not in self.allowed_modes:
            self.get_logger().error(f"Invalid Parameter. Must be one of these {self.allowed_modes}")
            return
        
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)
        self.handle_manual_mode_change()

        self.create_timer(0.1, self.laser_control_callback,)
        self.det_sub = self.create_subscription(Float64, '/detection', self.laser_control_callback, 10)

        self.add_on_set_parameters_callback(self.parameter_callback)
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'manual_mode':
                if param.value in self.allowed_modes:
                    self.manual_mode = param.value
                    self.handle_manual_mode_change()
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().error(f"Invalid Parameter. Must be one of these : {self.allowed_modes}")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=False)
    
    def handle_manual_mode_change(self):
        if self.manual_mode == 'MANUAL_ON':
            self.serial_port.write("MANUAL_ON\n".encode('utf-8'))
            self.get_logger().info("Manual Mode On")
        elif self.manual_mode == 'MANUAL_OFF':
            self.serial_port.write("MANUAL_OFF\n".encode('utf-8'))
            self.get_logger().info("Manual Mode Off")

    def laser_control_callback(self):    
        if self.manual_mode == 'MANUAL_OFF':
            self.serial_port.write(f"14, 30\n".encode('utf-8'))
            self.get_logger().info(f"Sent: 14, 30")
            time.sleep(0.3)
            self.serial_port.write(f"14, 0\n".encode('utf-8'))
            self.get_logger().info(f"Sent: 14, 0")
            time.sleep(0.3)
        
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').rstrip()
            self.get_logger().info(f'{line}')

def main(args=None):
    rclpy.init(args=args)
    node = LaserControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()