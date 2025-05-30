import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2 as cv
import time
import os

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')

        self.frame_sub = self.create_subscription(Image, "/view/frames", self.viewer_callback, 10)
        self.bbox_sub = self.create_subscription(Float64MultiArray, "/view/bbox_info", self.bbox_callback, 10)
        self.bridge = CvBridge()
        self.bbox = []

        # Initialize Timer
        self.bbox_timeout = 0.5
        self.last_bbox_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize FPS calculation
        self.last_frame_time = time.time()
        self.avg_fps = 0
        self.fps = []

        # Set up video writer
        self.video_writer = None
        self.recording_path = "/home/usrl1234/ros2_ws/src/drone_tracker/metrics_gathering/Video_Recordings/"
        self.create_video_writer()

        # Register shutdown callback
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def create_video_writer(self):
        # Ensure the recording directory exists
        if not os.path.exists(self.recording_path):
            os.makedirs(self.recording_path)
            self.get_logger().info(f"Created directory: {self.recording_path}")

        # Create a unique video file name using timestamp
        video_filename = time.strftime("tracking_%Y%m%d-%H%M%S.avi")
        full_path = os.path.join(self.recording_path, video_filename)
        self.get_logger().info(f"Saving video to: {full_path}")

        # Define video codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        # Initialize with default size, will update when we get the first frame
        self.video_writer = cv.VideoWriter(full_path, fourcc, 20.0, (640, 480))  # Placeholder size, update later

    def bbox_callback(self, bbox_msg: Float64MultiArray):
        self.bbox = bbox_msg.data
        x1, y1, x2, y2 = self.bbox
        self.last_bbox_time = self.get_clock().now()

    def viewer_callback(self, tracked_img: Image):
        current_time = time.time()
        self.fps.append(1.0 / (current_time - self.last_frame_time))
        self.avg_fps = sum(self.fps) / len(self.fps)
        self.last_frame_time = current_time

        frame = self.bridge.imgmsg_to_cv2(tracked_img, "bgr8")

        # Update the size of the VideoWriter when we receive the first frame
        if self.video_writer is None or not self.video_writer.isOpened():
            height, width = frame.shape[:2]
            fourcc = cv.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv.VideoWriter(os.path.join(self.recording_path, time.strftime("tracking_%Y%m%d-%H%M%S.avi")), fourcc, 20.0, (width, height))

        if len(self.bbox) == 4:
             current_time = self.get_clock().now()
             if (current_time - self.last_bbox_time).nanoseconds / 1e9 <= self.bbox_timeout:
                x1, y1, x2, y2 = map(int, self.bbox)
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.circle(frame, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 2, (0, 255, 0), 2)
                cv.putText(frame, f"({x1},{y1},{x2},{y2})", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        height, width = frame.shape[:2]
        # Draw vertical and horizontal lines
        cv.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
        cv.line(frame, (0, height // 2), (width, height // 2), (255, 255, 255), 1)

        # Save frame to video file
        if self.video_writer is not None and self.video_writer.isOpened():
            self.video_writer.write(frame)

        cv.imshow("Location Prediction", frame)
        cv.waitKey(1)

    def timer_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_bbox_time).nanoseconds / 1e9 > self.bbox_timeout:
            self.bbox = []  # Clear bbox if timeout period has passed

    def on_shutdown(self):
        self.get_logger().info("Shutting down Viewer node.")
        if self.video_writer is not None:
            self.video_writer.release()  # Finalize the video file properly
            self.get_logger().info("Video saved successfully.")
        cv.destroyAllWindows()  # Close all OpenCV windows

def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    rclpy.spin(node)
    rclpy.shutdown()
