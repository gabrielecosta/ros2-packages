import rclpy
import cv2 as cv
import numpy as np
from aruco_detector.arucoUtils import arucoUtils
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
# from aruco_detector.msg import ArucoMarkers, ArucoMarker  # Import messaggi custom

class ArucoDisplayer(Node):
    
    def __init__(self):
        super().__init__("aruco_displayer")
        self.cv_bridge = CvBridge()
        self.subscription_ = self.create_subscription(String, "/aruco_detection", self.displayCallback, 10)
        self.frame_count = 0

    
    def displayCallback(self, img_msg : String):
        self.frame_count += 1
        self.get_logger().info(f"The image number {self.frame_counter} has been read")
        # Converting the string received into a dictionary
        marker_data = json.loads(img_msg.data)
        # Accessing the dictionary keys
        marker_id = marker_data.get('markerID')
        c_x = marker_data.get('cX')
        c_y = marker_data.get('cY')
        angle = marker_data.get('angle')
        self.get_logger().info(f"Marker ID: {marker_id}, cX: {c_x}, cY: {c_y}, angle: {angle}")


def main(args=None):
    rclpy.init(args=args)
    aruco_detector_node = ArucoDisplayer()
    rclpy.spin(aruco_detector_node)
    aruco_detector_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()