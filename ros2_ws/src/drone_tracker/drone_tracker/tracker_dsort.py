import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
import time

class Tracker(Node):
    """
        Tracks drones in images
    """
    def __init__(self):
        super().__init__('drone_tracker')
        
        self.get_logger().info(f"Tracker mode : deep sort")
        
        '''SETS UP TRACKER'''
        #Set up variables
        self.detections = None
        self.has_tracked = False                
        
        self.tracker = DeepSort(max_age=30, n_init=3, nn_budget=100, max_iou_distance=0.7)
        self.last_detection_time = time.time()
        self.timeout = 2
        self.count = 0
        
        '''PUBLISHERS AND SUBSCRIBERS'''
        self.frame_sub = self.create_subscription(Image, "/detection/frames", self.frame_callback, 5)
        self.det_sub = self.create_subscription(Float64MultiArray, "/detection/boxes", self.detection_callback, 10)
        self.coords_pub = self.create_publisher(Float64MultiArray, "/view/bbox_info", 20)
        self.tracker_frame_pub = self.create_publisher(Image, "/view/frames", 20)
        #Data Gathering
        self.data_bbox_pub = self.create_publisher(Float64MultiArray, "/data/bbox", 20)
        
        self.bridge = CvBridge()

    def frame_callback(self, detection_img:Image):
        frame = self.bridge.imgmsg_to_cv2(detection_img, "bgr8")
        # self.count += 1
        # self.get_logger().info(f"{self.count}")

        if time.time() - self.last_detection_time <= self.timeout:
            if self.detections is not None:
                self.track_with_deep_sort(frame)
                self.has_tracked = True
        if time.time() - self.last_detection_time > self.timeout and self.has_tracked:
            self.get_logger().info(f"Tracker reset due to failure for more than {self.timeout} seconds.")
            self.has_tracked = False
        if not self.has_tracked:
            self.publish_bbox(0,0,0,0)

        tracked_img = self.bridge.cv2_to_imgmsg(frame, "bgr8") 
        self.tracker_frame_pub.publish(tracked_img)

    def track_with_deep_sort(self, frame): 
        bbs = []

        if len(self.detections) > 0:
            # Sort detections by confidence and pick the highest confidence detection
            self.detections = sorted(self.detections, key=lambda x: x[4], reverse=True)
            best_detection = self.detections[0]
            x1, y1, x2, y2, conf, cls = best_detection
            w = x2 - x1
            h = y2 - y1
            bbs.append(([x1, y1, w, h], conf, cls))

        tracks = self.tracker.update_tracks(bbs, frame=frame)

        l = None
        for track in tracks:
            if track.is_confirmed():
                l, t, r, b = track.to_ltrb()
                self.publish_bbox(l, t, r, b)
                break  # Only process the first confirmed track (highest confidence)

        if l is None:
            self.publish_bbox(0, 0, 0, 0)


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
