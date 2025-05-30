import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import subprocess, os, keyboard, time, serial
import numpy as np


class Movement(Node):
    """
        Moves tracker
    """
    def __init__(self):
        super().__init__('telescope_movement')

        try:
            self.ser = serial.Serial(port="COM4",baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
            self.get_logger().info(f'Port Details -> {self.ser}')
    
        except serial.SerialException:
            self.get_logger().info('Port not available')

        #PID AZM PARAMETERS
        self.declare_parameter('kp_azm', 0.1)
        self.declare_parameter('ki_azm', 0.01)
        self.declare_parameter('kd_azm', 0.01)

        self.kp_azm = self.get_parameter('kp_azm').value
        self.ki_azm = self.get_parameter('ki_azm').value
        self.kd_azm = self.get_parameter('kd_azm').value

        #PID ALT PARAMETERS
        self.declare_parameter('kp_alt', 0.1)
        self.declare_parameter('ki_alt', 0.01)
        self.declare_parameter('kd_alt', 0.01)

        self.kp_alt = self.get_parameter('kp_alt').value
        self.ki_alt = self.get_parameter('ki_alt').value
        self.kd_alt = self.get_parameter('kd_alt').value

        #PID Initialization - Declared Variables
        self.target_azm, self.target_alt = 0.0, 0.0
        self.error_sum_azm, self.error_sum_alt = 0.0, 0.0
        self.last_error_azm, self.last_error_alt = 0.0, 0.0
        self.last_time = self.get_clock().now()

        #Other Declared Variables
        self.bbox_c = None
        self.frame_c = None

        #Backlash Correction
        self.previous_control_azm = 0.0 
        self.previous_control_alt = 0.0
    
        self.subscription = self.create_subscription(Float64MultiArray, '/bbox_info', self.bbox_to_center, 10)
        self.frame_sub = self.create_subscription(Image, "/tracker/frames", self.frame_center, 10)
        
        self.bridge = CvBridge()
        
    def bbox_to_center(self, bbox_msg:Float64MultiArray):
        x1, y1, x2, y2 = bbox_msg.data
        self.bbox_c = ((x2 - x1)/2 + x1, (y2 - y1)/2 + y1)
        self.calc_arc_change()

    def frame_center(self, tracked_img: Image):
        frame = self.bridge.imgmsg_to_cv2(tracked_img, "bgr8")
        height, width = frame.shape[:2]
        self.frame_c = (width / 2, height / 2)

    def calc_arc_change(self):
        if self.bbox_c is None or self.frame_c is None:
            return #Wait until both msgs recieved
        
        change_x, change_y = self.bbox_c[0] - self.frame_c[0], self.bbox_c[1] - self.frame_c[1]

        #Camera Properties
        fx = 800 #focal length x
        fy = 800 #focal length y
        cx = self.frame_c[0]
        cy = self.frame_c[1]
        zoom = 2

        theta_azm, theta_alt = self.pixel_to_angle(change_x, change_y, fx, fy, cx, cy, zoom)

        if theta_azm > 1:
            self.pid_azm(theta_azm)
        if theta_alt > 1:
            self.pid_alt(theta_alt)

    def pixel_to_angle(self, delta_x, delta_y, fx, fy, cx, cy, zoom):
        
        fx_prime = zoom * fx
        fy_prime = zoom * fy

        x_normalized = (delta_x - cx) / fx_prime
        y_normalized = (delta_y - cy) / fy_prime

        theta_azm = np.arctan(x_normalized)
        theta_alt = np.arctan(y_normalized)

        return theta_azm, theta_alt

    def pid_azm(self, theta_azm):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        error = self.target_azm - theta_azm
        self.error_sum_azm += error * dt
        d_error = (error - self.last_error_azm) / dt

        control = self.kp_azm * error + self.ki_azm * self.error_sum_azm + self.kd_azm * d_error

        self.last_error_azm = error
        self.last_time = current_time

        self.speed_azm(control)
    
    def pid_alt(self, theta_alt):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        error = self.target_alt - theta_alt
        self.error_sum_alt += error * dt
        d_error = (error - self.last_error_alt) / dt

        control = self.kp_alt * error + self.ki_alt * self.error_sum_alt + self.kd_alt * d_error

        self.last_error_alt = error
        self.last_time = current_time

        self.speed_alt(control)

    def speed_azm(self, control):
        desired_deg_rate_azm = control * (180 / np.pi)  # Convert radians to degrees
        desired_rate_azm = desired_deg_rate_azm * 3600  # Convert to arcseconds/sec
        trackRateHigh_azm = int((desired_rate_azm * 4) // 256)
        trackRateLow_azm = int((desired_rate_azm * 4) % 256)

        if control > 0:
            # Positive direction
            chars = ['P', chr(3), chr(16), chr(6), chr(trackRateHigh_azm), chr(trackRateLow_azm), chr(0), chr(0)]
        else:
            # Negative direction
            chars = ['P', chr(3), chr(16), chr(7), chr(trackRateHigh_azm), chr(trackRateLow_azm), chr(0), chr(0)]

        hex_output = [hex(ord(char)) for char in chars]

    def speed_alt(self, control):
        desired_deg_rate_alt = control * (180 / np.pi)  # Convert radians to degrees
        desired_rate_alt = desired_deg_rate_alt * 3600  # Convert to arcseconds/sec
        trackRateHigh_alt = int((desired_rate_alt * 4) // 256)
        trackRateLow_alt = int((desired_rate_alt * 4) % 256)

        if control > 0:
            # Positive direction
            chars = ['P', chr(3), chr(17), chr(6), chr(trackRateHigh_alt), chr(trackRateLow_alt), chr(0), chr(0)]
        else:
            # Negative direction
            chars = ['P', chr(3), chr(17), chr(7), chr(trackRateHigh_alt), chr(trackRateLow_alt), chr(0), chr(0)]

        hex_output = [hex(ord(char)) for char in chars]
    
    def live_pid_graphing(self):
        pass
    #Implementation as to change the PID in realtime



def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    rclpy.shutdown()