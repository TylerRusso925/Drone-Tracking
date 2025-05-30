import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np 
import time
from .iou_get import get_max_iou_detection

class Tracker(Node):
    """
        Tracks drones in images
    """
    def __init__(self):
        super().__init__('drone_tracker')

        '''SET TRACKER MODE PARAMETERS'''
        self.declare_parameter('tracker_mode', 'kcf')

        self.tracker_mode = self.get_parameter('tracker_mode').get_parameter_value().string_value

        allowed_modes = ['csrt', 'mosse', 'kcf']

        if self.tracker_mode not in allowed_modes:
            self.get_logger().error(f"Invalid operation_mode: {self.tracker_mode}. Must be one of {allowed_modes}")
            return
        self.get_logger().info(f"Tracker mode : {self.tracker_mode}")

        '''SETS UP TRACKER'''
        #Set up variables
        self.bbox = None
        self.timeout = 2
        self.frame = None
        self.last_success_time = time.time()
        self.tracker = self.create_tracker(self.tracker_mode)
        self.has_tracked = False
        # self.count = 0
        
        '''PUBLISHERS AND SUBSCRIBERS'''
        self.frame_sub = self.create_subscription(Image, "/detection/frames", self.frame_callback, 100)
        self.det_sub = self.create_subscription(Float64MultiArray, "/detection/boxes", self.detection_callback, 100)
        self.coords_pub = self.create_publisher(Float64MultiArray, "/view/bbox_info", 10)
        self.tracker_frame_pub = self.create_publisher(Image, "/view/frames", 10)
        #Data Gathering
        self.data_bbox_pub = self.create_publisher(Float64MultiArray, "/data/bbox",10)
        
        self.bridge = CvBridge()

        #Resetting Tracker if drone lost
        self.reset_timer = self.create_timer(0.1, self.check_tracking_status)
        
    def create_tracker(self, mode: str):
        if mode == 'mosse':
            return cv.legacy.TrackerMOSSE.create()
        elif mode == 'csrt':
            return cv.legacy.TrackerCSRT.create()
        elif mode == 'kcf':
            return cv.legacy.TrackerKCF.create()


    
    def frame_callback(self, detection_img:Image):
        self.frame = self.bridge.imgmsg_to_cv2(detection_img, "bgr8")
        # self.count += 1
        # self.get_logger().info(f"{self.count}")
        if self.bbox is not None:
            success, self.bbox = self.tracker.update(self.frame)
            if success:
                self.publish_center_coords()
            else:
                self.bbox = [0,0,0,0]
                self.publish_center_coords()
                self.bbox = None
        else:
            self.bbox = [0,0,0,0]
            self.publish_center_coords()
            self.bbox = None

        self.publish_frame()


    def detection_callback(self, bbox_info: Float64MultiArray):
        if self.frame is None:
            self.get_logger().warning("No frame available for detection.")
            return

        self.detections = np.array(bbox_info.data).reshape(-1, 6)
        if len(self.detections) > 0:
            close_det = self.get_closest_detection(self.detections)
            self.bbox = (close_det[0], close_det[1], close_det[2] - close_det[0], close_det[3] - close_det[1])
            self.tracker = self.create_tracker(self.tracker_mode)
            self.tracker.init(self.frame, self.bbox)
            self.last_success_time = time.time()
            self.has_tracked = True
    
    def get_closest_detection(self, detections):
        if self.bbox is not None:
            prev_bbox = self.bbox
            close_det = get_max_iou_detection(detections, prev_bbox)
        else:
            close_det = max(detections, key=lambda det: det[4])
            # self.get_logger().info(f"{self.bbox}")
        return close_det

    def check_tracking_status(self):
        if (time.time() - self.last_success_time > self.timeout) and self.has_tracked:
            self.get_logger().info(f"Tracker reset due to failure for more than {self.timeout} seconds.")
            self.bbox = None
            self.detections = None
            self.has_tracked = False
    
    def publish_center_coords(self):
        x1, y1, w, h = self.bbox
        x2, y2 = x1 + w, y1 + h
        bbox_msg = Float64MultiArray(data=[x1, y1, x2, y2])
        # self.get_logger().info(f"x1: {x1} | y1: {y1} | x2: {x2} | y2: {y2}")
        self.coords_pub.publish(bbox_msg)
        self.data_bbox_pub.publish(bbox_msg)
    
    def publish_frame(self):
        tracked_img = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        self.tracker_frame_pub.publish(tracked_img)


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    rclpy.shutdown()