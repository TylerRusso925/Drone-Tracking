import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time


class VideoPlayer(Node):

    def __init__(self):
        super().__init__('telescope_images')

        self.declare_parameter('video_index', 0)
        video_index = self.get_parameter('video_index').get_parameter_value().integer_value
        videos = ["video01", "video02", "video03", "video04", "video05"]
        self.get_logger().info(f"Playing {videos[video_index]}")
        video_directory = "/home/usrl1234/Desktop/Testing_Trackers/Anti-UAV-Tracking-V0/" + videos[video_index]

        self.image_files = sorted([os.path.join(video_directory, f) for f in os.listdir(video_directory) if f.endswith('.jpg')])

        self.frame_pub = self.create_publisher(Image, '/frames', 50)
        self.bridge = CvBridge()
        self.current_frame = 0
        self.actual = 0
        self.start_time = 0

        self.timer = self.create_timer(0.25, self.send_frames)  


    def send_frames(self):
        if self.current_frame < len(self.image_files):
            frame = cv.imread(self.image_files[self.current_frame])
            img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.frame_pub.publish(img)
            self.current_frame += 1
        elif self.current_frame == len(self.image_files):
            self.current_frame += 1
            self.get_logger().info("Last Frame Published")


def main(args=None):

    rclpy.init(args=args)
    node = VideoPlayer()
    rclpy.spin(node)
