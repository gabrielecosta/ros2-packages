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

class ArucoDetectorNode(Node):
    
    def __init__(self):
        super().__init__("aruco_detector")
        self.cv_bridge = CvBridge()
        self.subscription_ = self.create_subscription(Image, "/image_feed_topic", self.detectCallback, 10)
        self.frame_count = 0
        # self.timer_ = self.create_timer(1.0 / 30.0, self.publishCallback)
        self.publisher_ = self.create_publisher(String, '/aruco_detection', 20)
        self.path_param = './parameters/calibration_params.npz'
        self.mtx, self.dist = self.loadParam(self.path_param)
        self.aruco_utils = arucoUtils()
        self.aruco_dict = self.aruco_utils.getAruco_dict()
        self.parameters = self.aruco_utils.getParameters()

    def loadParam(self, path_param):
        # Load previously saved data
        with np.load(path_param) as data:
            mtx, dist = [data[i] for i in ('camera_matrix', 'dist_coeffs')]
        return mtx, dist
    
    def undistortImage(self, frame):
        # dimensioni frame originale
        h, w = frame.shape[:2]
        frame_size = (w,h)
        #applico la undistort
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        # trovo le aree di interesse e faccio crop
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        resized_img = cv.resize(dst, frame_size, interpolation=cv.INTER_LANCZOS4)  # For high-quality resizing
        return resized_img
    
    def detectCallback(self, img_msg : Image):
        self.frame_count += 1
        self.get_logger().info(f"The image number {self.frame_counter} has been read")
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        cv_image_und = self.undistortImage(cv_image)
        self.publishCallback(cv_image_und)

    def publishCallback(self, frame):
        '''
        prende il frame, poi trova gli aruco con getArucos e infine
        li pubblica su un topic, stampando quindi una stringa contenente
        gli aruco trovati in quell'istante con la relativa posizione
        Hint: provare a separare aruco displayer con aruco finder, magari facendo in
        modo che leggo tramite JSON, deserializzo e infine faccio display del frame
        '''
        corners, ids, rejected = cv.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        # detected_markers, vectors_markers = self.aruco_utils.aruco_display(corners, ids, rejected, frame)
        vectors_markers = self.aruco_utils.aruco_vector_state(corners, ids)
        marked_frame = self.aruco_utils.aruco_display(corners, ids, rejected, frame)
        # Convert the marker list into the desired format (id, x, y, angle)
        marker_list = [{'id': marker[0], 'x': marker[1], 'y': marker[2], 'angle': marker[3]} for marker in vectors_markers]
        # Serialize the list to a JSON string for easier transmission
        marker_message = json.dumps(marker_list)
        # Publish the marker list on aruco_markers_topic
        self.publisher_.publish(String(data=marker_message))
        self.get_logger().info(f"At frame #{self.frame_count} I have found this markers: {vectors_markers}")
        cv.imshow('Aruco detected', marked_frame)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector_node = ArucoDetectorNode()
    rclpy.spin(aruco_detector_node)
    aruco_detector_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()