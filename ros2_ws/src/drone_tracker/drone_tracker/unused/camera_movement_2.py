import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
import Jetson.GPIO as GPIO 
import time
import matplotlib.pyplot as plt
import threading

output_pins = [15, 33]

class Movement(Node):
    """
        Moves tracker
    """
    def __init__(self):
        super().__init__('cam_movement')

        self.declare_parameters(namespace='', parameters=[('kp_azm',0.01), ('ki_azm',0.0), ('kd_azm',0.0),('kp_alt',0.01), ('ki_alt',0.0), ('kd_alt',0.0)])
        self.initialize_variables()
        self.parameters()
        self.setup_plotting()
        self.subscription = self.create_subscription(Float64MultiArray, '/bbox_info', self.bbox_to_center, 10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.plot_timer = self.create_timer(0.1, self.update_plot)

    def parameters(self):

        parameters = self.get_parameters(['kp_azm','ki_azm','kd_azm','kp_alt','ki_alt','kd_alt'])

        self.kp_azm = parameters[0].value
        self.ki_azm = parameters[1].value
        self.kd_azm = parameters[2].value
        self.kp_alt = parameters[3].value
        self.ki_alt = parameters[4].value
        self.kd_alt = parameters[5].value

        self.record_gain_history()

    def initialize_variables(self):

        #Necessary Variables
        self.target_azm, self.target_alt = 0.0, 0.0
        self.error_sum_azm, self.error_sum_alt = 0.0, 0.0
        self.last_error_azm, self.last_error_alt = 0.0, 0.0
        self.control_angle_alt, self.control_angle_azm = 90, 90
        self.tolerance_azm, self.tolerance_alt = 0.5, 0.5
        self.last_time = self.get_clock().now()
        self.bbox_c = (640/2, 480/2)

        #Data for plotting
        self.errors_x, self.errors_y = [], []
        self.control_azms, self.control_alts = [], []
        self.times = []
        self.kp_azm_history, self.ki_azm_history, self.kd_azm_history = [], [], []
        self.kp_alt_history, self.ki_alt_history, self.kd_alt_history = [], [], []


    def setup_plotting(self):
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        plt.ion()
        plt.show()

    def parameter_callback(self, parameters):
        result = SetParametersResult(successful=True)

        for param in parameters:
            if param.name in ('kp_azm', 'ki_azm', 'kd_azm','kp_alt','ki_alt','kd_alt'):
                if param.type_ != Parameter.Type.DOUBLE:
                    result.successful = False
                    result.reason = f'{param} must be a float'
                    return result
            setattr(self, param.name, param.value)
            self.record_gain_history()
            
        return result
        
    def bbox_to_center(self, bbox_msg:Float64MultiArray):
        x1, y1, x2, y2 = bbox_msg.data
        self.bbox_c = ((x2 - x1)/2 + x1, (y2 - y1)/2 + y1)
        self.calc_arc_change()
        self.bbox_c = (640/2, 480/2)

    def calc_arc_change(self):
        #Camera Properties
        horizontal_fov, vertical_fov = 69, 42  # degrees
        cx , cy = 640, 480 

        change_x, change_y = (self.bbox_c[0] - cx/2, self.bbox_c[1] - cy/2)
        deg_azm = (change_x / cx) * horizontal_fov
        deg_alt = -(change_y / cy) * vertical_fov

        self.get_logger().info(f"deg_azm : {deg_azm} |  deg_alt : {deg_alt}")
        self.pid(deg_azm, deg_alt)

    def pid(self, theta_azm, theta_alt):
        self.parameters()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        error_azm, error_alt = self.target_azm - theta_azm, self.target_alt - theta_alt

        #Dead Zone (When within this area, it does not move it)
        if abs(error_azm) < self.tolerance_azm:
            error_azm = 0
        if abs(error_alt) < self.tolerance_alt:
            error_alt = 0    
        
        self.error_sum_azm += error_azm * dt
        self.error_sum_alt += error_alt * dt
        d_error_azm = (error_azm - self.last_error_azm) / dt
        d_error_alt = (error_alt - self.last_error_alt) / dt

        control_azm = self.kp_azm * error_azm + self.ki_azm * self.error_sum_azm + self.kd_azm * d_error_azm
        control_alt = self.kp_alt * error_alt + self.ki_alt * self.error_sum_alt + self.kd_alt * d_error_alt

        self.get_logger().info(f"Control Azm: {control_azm} | Control Alt: {control_alt}")
        self.update_servo(control_azm, control_alt)
        self.record_data(current_time, error_azm, error_alt, control_azm, control_alt)

    def record_data(self, current_time, error_azm, error_alt, control_azm, control_alt):
        #Store data for plotting
        self.times.append(time.time())
        self.errors_x.append(error_azm)
        self.errors_y.append(error_alt)
        self.control_azms.append(control_azm)
        self.control_alts.append(control_alt)

        self.last_error_azm, self.last_error_alt = error_azm, error_alt
        self.last_time = current_time
    
    def record_gain_history(self):
        self.kp_azm_history.append(self.kp_azm)
        self.ki_azm_history.append(self.ki_azm)
        self.kd_azm_history.append(self.kd_azm)
        self.kp_alt_history.append(self.kp_alt)
        self.ki_alt_history.append(self.ki_alt)
        self.kd_alt_history.append(self.kd_alt)

    def update_servo(self, control_azm, control_alt):
        self.control_angle_azm = max(0, min(180, self.control_angle_azm + control_azm))
        self.control_angle_alt = max(0, min(180, self.control_angle_alt + control_alt))
        
        duty_cycle_azm = 2.5 + (self.control_angle_azm / 18)
        duty_cycle_alt = 2.5 + (self.control_angle_alt / 18)
        
        self.azm_pwm.ChangeDutyCycle(duty_cycle_azm)
        self.alt_pwm.ChangeDutyCycle(duty_cycle_alt)
        self.get_logger().info(f"AZM C: {control_azm} | ALT C:{control_alt} | "
                               f"Azm: {self.control_angle_azm} degrees | Alt: {self.control_angle_alt} degrees")
        time.sleep(0.02)
    
    def update_plot(self):
        self.ax1.clear()
        self.ax1.plot(self.times, self.errors_x, label='Horizontal Error (X)')
        self.ax1.plot(self.times, self.errors_y, label='Vertical Error (Y)')
        self.ax1.set_ylabel('Error (degrees)')
        self.ax1.legend()
        self.ax1.grid(True)
        
        self.ax2.clear()
        self.ax2.plot(self.times, self.control_azms, label='Pan Control Output')
        self.ax2.plot(self.times, self.control_alts, label='Tilt Control Output')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Control Output (degrees)')
        self.ax2.legend()
        self.ax2.grid(True)

        self.ax3.clear()
        updates = range(len(self.kp_azm_history))
        self.ax3.plot(updates, self.kp_azm_history, label='KP_AZM')
        self.ax3.plot(updates, self.ki_azm_history, label='KI_AZM')
        self.ax3.plot(updates, self.kd_azm_history, label='KD_AZM')
        self.ax3.plot(updates, self.kp_alt_history, label='KP_ALT')
        self.ax3.plot(updates, self.ki_alt_history, label='KI_ALT')
        self.ax3.plot(updates, self.kd_alt_history, label='KD_ALT')
        self.ax3.set_xlabel('Update Index')
        self.ax3.set_ylabel('Control Gains')
        self.ax3.legend()
        self.ax3.grid(True)     

        plt.pause(0.001)

    def shutdown(self):
        try:
            # Stop PWM channels explicitly
            if self.azm_pwm:
                self.azm_pwm.stop()
                self.get_logger().info("AZM PWM stopped successfully.")
            if self.alt_pwm:
                self.alt_pwm.stop()
                self.get_logger().info("ALT PWM stopped successfully.")

            # Cleanup GPIO
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            # Ensure that any additional cleanup or closing operations are performed
            plt.ioff()

    def start_GPIO(self, pin):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        if pin == 15:
            self.azm_pwm = GPIO.PWM(pin, 50)
            self.azm_pwm.start(7.5)
        if pin == 33:
            self.alt_pwm = GPIO.PWM(pin, 50)
            self.alt_pwm.start(7.5)

def main(args=None):
    rclpy.init(args=args)
    GPIO.setmode(GPIO.BOARD)
    node = Movement()
    threads = []
    for pin in output_pins:
        thread = threading.Thread(target=node.start_GPIO, args=(pin,))
        threads.append(thread)
        thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        plt.savefig('plot.png')
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        for thread in threads:
            thread.join()