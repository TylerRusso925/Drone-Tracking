import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from .iou_get import get_max_iou_detection 
import threading
import cv2 as cv

class Detection(Node):
    """
        Detects drones in images
    """
    def __init__(self):
        super().__init__('drone_detector')

        # Get the path to the model file
        package_share_directory = get_package_share_directory('drone_tracker')
        model_path = os.path.join(package_share_directory, 'models', 'size_t.engine')

        # Initialize YOLOv8 Drone Model
        self.model = YOLO(model_path)
        self.get_logger().info("Detection Model Loaded")
        self.bridge = CvBridge()
        self.tracker = cv.legacy.TrackerKCF.create()



        #Subscries to Frames
        self.frame_sub = self.create_subscription(Image, "/frames", self.frame_callback, 50)

        #Publish Frame and BBOX
        self.track_bbox_info_pub = self.create_publisher(Float64MultiArray, "/view/bbox_info", 10)
        self.track_frame_pub = self.create_publisher(Image, "/view/frames", 10)

        #Data Gathering
        # self.data_bbox_pub = self.create_publisher(Float64MultiArray, "/data/bbox",10)
        
        #KCF Initializer
        self.tracking_active = False
        self.bbox = None
        self.frame_count = 0
        self.stop_frame_amount = 20


    def frame_callback(self, img: Image):
        frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
        threading.Thread(target=self.process_frame, args=(frame,)).start()

    def process_frame(self, frame):
        """
        Process each frame to detect and track the UAV
        """
        results = self.model.predict(frame, conf = 0.7, device = 0)
        for r in results:
            boxes = r.boxes.data.cpu().numpy()
            if boxes.size > 0:
                if self.bbox is not None:
                    prev_bbox = self.bbox
                    close_box = get_max_iou_detection(boxes, prev_bbox)
                    self.bbox = [close_box[0],close_box[1],close_box[2],close_box[3]]
                else:
                    close_box = boxes[np.argmax(boxes[:,4])]
                    self.bbox = [close_box[0],close_box[1],close_box[2],close_box[3]]

                #Deletes and intializes new tracker everytime YOLO success
                self.tracker.clear()
                self.tracker = cv.legacy.TrackerKCF.create()
                self.tracker.init(frame, self.bbox)
                self.frame_count = 0
                self.publish_bbox_and_frame(frame, self.bbox)

            #Active KCF for 20 Frames of missed YOLO detections
            else:
                if self.frame_count <= self.stop_frame_amount:
                    self.tracking_active = True
                    self.frame_count += 1
        if self.tracking_active:
            if self.bbox is not None:   #MAKES SURE YOLO has had at least a success
                success, self.bbox = self.tracker.update(frame)
                if success:
                    self.publish_bbox_and_frame(frame, self.bbox)
                    self.tracking_active = False
            else:
                pass


    def publish_bbox_and_frame(self, frame, bbox):
        x1, y1, w, h = bbox
        x2, y2 = x1 + w, y1 + h
        bbox_msg = Float64MultiArray(data=[x1,y1,x2,y2])
        tracked_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.track_frame_pub.publish(tracked_img)
        self.track_bbox_info_pub.publish(bbox_msg)
        



def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    rclpy.spin(node)
    rclpy.shutdown()
