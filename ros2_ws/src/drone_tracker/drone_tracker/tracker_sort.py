import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from .sort import *

class Tracker(Node):
    """
        Tracks drones in images
    """
    def __init__(self):
        super().__init__('drone_tracker')
        
        self.get_logger().info(f"Tracker mode : sort")
        
        '''SETS UP TRACKER'''
        #Set up variables
        self.detections = None
        self.tracked_object = None
        self.has_tracked = False                
        
        self.tracker = Sort(max_age=30, min_hits=3, iou_threshold=0.3)
        self.last_detection_time = time.time()
        self.timeout = 2
        self.count = 0

        '''PUBLISHERS AND SUBSCRIBERS'''
        self.frame_sub = self.create_subscription(Image, "/detection/frames", self.frame_callback, 10)
        self.det_sub = self.create_subscription(Float64MultiArray, "/detection/boxes", self.detection_callback, 10)
        self.coords_pub = self.create_publisher(Float64MultiArray, "/view/bbox_info", 20)
        self.tracker_frame_pub = self.create_publisher(Image, "/view/frames", 20)
        #Data Gathering
        self.data_bbox_pub = self.create_publisher(Float64MultiArray, "/data/bbox",10)
        
        self.bridge = CvBridge()

    def frame_callback(self, detection_img:Image):
        frame = self.bridge.imgmsg_to_cv2(detection_img, "bgr8")
        # self.count += 1
        # self.get_logger().info(f"{self.count}")
        
        if time.time() - self.last_detection_time <= self.timeout:
            if self.detections is not None:
                self.track_with_sort()
                self.has_tracked = True
        if time.time() - self.last_detection_time > self.timeout and self.has_tracked:
            self.get_logger().info(f"Tracker reset due to failure for more than {self.timeout} seconds.")
            self.has_tracked = False
        if not self.has_tracked:
            self.publish_bbox(0,0,0,0)

        tracked_img = self.bridge.cv2_to_imgmsg(frame, "bgr8") 
        self.tracker_frame_pub.publish(tracked_img)

    def track_with_sort(self):
        x1 = None
        trackers = self.tracker.update(self.detections)
        if len(trackers) > 0:
            self.tracked_object = trackers[0]  # We only track the first object
            x1, y1, x2, y2, obj_id = self.tracked_object
            self.publish_bbox(x1, y1, x2, y2)
        
        if x1 == None:
            self.publish_bbox(0,0,0,0)

    def detection_callback(self, box_info: Float64MultiArray):
        self.detections = np.array(box_info.data).reshape(-1, 6)
        self.last_detection_time = time.time()

    def publish_bbox(self, x1, y1, x2, y2):
        bbox_msg = Float64MultiArray(data=[x1, y1, x2, y2])
        # self.get_logger().info(f"x1: {x1} | y1: {y1} | x2: {x2} | y2: {y2}")
        self.coords_pub.publish(bbox_msg)
        self.data_bbox_pub.publish(bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    rclpy.shutdown()
