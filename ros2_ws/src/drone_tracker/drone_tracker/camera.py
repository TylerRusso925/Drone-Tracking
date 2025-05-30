import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TelescopeView(Node):

    def __init__(self):
        super().__init__('telescope_images')
        self.frame_pub = self.create_publisher(Image, '/frames', 1)
        self.timer = self.create_timer(0.1, self.send_frames) # 30 Hz

        self.cap = cv.VideoCapture(4)
        self.bridge = CvBridge()
    
    def send_frames(self):

        success, frame = self.cap.read()
        if success:
        # Publish the frame
            img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.frame_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = TelescopeView()
    rclpy.spin(node)
    rclpy.shutdown()