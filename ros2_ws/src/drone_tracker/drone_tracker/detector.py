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

class Detection(Node):
    """
        Detects drones in images
    """
    def __init__(self):
        super().__init__('drone_detector')

        #Declare and obtain model size parameter
        self.declare_parameter('tracker_mode', 'mosse') 

        self.tracker_mode = self.get_parameter('tracker_mode').get_parameter_value().string_value

        # Get the path to the model file
        package_share_directory = get_package_share_directory('drone_tracker')
        model_path = os.path.join(package_share_directory, 'models', 'size_t.engine')

        # Initialize YOLOv8 Drone Model
        self.model = YOLO(model_path)
        self.get_logger().info("Detection Model Loaded")
        self.bridge = CvBridge()



        # Subscribe to frames
        self.frame_sub = self.create_subscription(Image, "/frames", self.frame_callback, 50)

        #For Publishing to Trackers
        self.frame_pub = self.create_publisher(Image, "/detection/frames", 10)
        self.det_bbox_info_pub = self.create_publisher(Float64MultiArray, "/detection/boxes", 10)

        #For Publishing with no Trackers
        self.track_bbox_info_pub = self.create_publisher(Float64MultiArray, "/view/bbox_info", 10)
        self.track_frame_pub = self.create_publisher(Image, "/view/frames", 10)

        #Data Gathering
        self.data_bbox_pub = self.create_publisher(Float64MultiArray, "/data/bbox",10)
        #Global Class Variables
        self.bbox = []



    def frame_callback(self, img: Image):
        frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
        threading.Thread(target=self.process_frame, args=(frame,)).start()

    def process_frame(self, frame):
        bbox_msg = None
        results = self.model.predict(frame, conf=0.7, device=0)
        for r in results:
            boxes = r.boxes.data.cpu().numpy()
            if boxes.size > 0:
                box_info = Float64MultiArray(data=boxes.flatten().tolist())
                if self.tracker_mode == 'na':
                    detections = np.array(boxes).reshape(-1,6)
                    if self.bbox == 4:
                        prev_bbox = self.bbox
                        if detections[:,4] > 0.5:
                            close_det = get_max_iou_detection(detections, prev_bbox)
                    else:
                        if detections[:, 4].max() > 0.5:
                            close_det = detections[np.argmax(detections[:, 4])]
                            self.bbox = [close_det[0], close_det[1], close_det[2], close_det[3]]
                    
                    bbox_msg = Float64MultiArray(data=self.bbox)
                    self.track_bbox_info_pub.publish(bbox_msg)

                else:
                    self.det_bbox_info_pub.publish(box_info)


        if self.tracker_mode == 'na':
            tracked_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.track_frame_pub.publish(tracked_img)
            if bbox_msg:
                self.data_bbox_pub.publish(bbox_msg)
            else:
                bbox_msg = Float64MultiArray(data=[0,0,0,0])
                self.data_bbox_pub.publish(bbox_msg)
        
        else:
            detection_img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.frame_pub.publish(detection_img)
        
        

        



def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    rclpy.spin(node)
    rclpy.shutdown()
