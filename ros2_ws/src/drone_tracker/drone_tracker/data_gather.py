import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os


class BagRecorder(Node):
    def __init__(self):
        super().__init__('Frame_Info_Recorder')
        self.declare_parameter('video_index', 0)
        video_index = self.get_parameter('video_index').get_parameter_value().integer_value

        self.declare_parameter('tracker_mode', 'mosse') 

        tracker_mode = self.get_parameter('tracker_mode').get_parameter_value().string_value
        #Subscribe to frames + bbox info
        #Data Gathering
        self.data_bbox_pub = self.create_subscription(Float64MultiArray, "/data/bbox", self.bbox_to_csv, 50)

        #CSV File Initialization
        self.csv_file_path = os.path.join(os.getcwd(), f"{video_index}_{tracker_mode}.csv")

        # If the file exists, clear its contents
        if os.path.exists(self.csv_file_path):
            open(self.csv_file_path, 'w').close()  # This opens the file in write mode, automatically clearing its contents


    def bbox_to_csv(self, bbox_msg: Float64MultiArray):
        bbox_values = bbox_msg.data

        # Log the bbox info to the CSV file
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(bbox_values)


    
def main(args=None):
    rclpy.init(args=args)
    sbr = BagRecorder()
    rclpy.spin(sbr)
    sbr.destroy_node()
    rclpy.shutdown()


