import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SetupTest(Node):

    def __init__(self):
        super().__init__("setup_test_node")
        self.cv_bridge = CvBridge()
        self.subscription_ = self.create_subscription(Image,"/image_feed_topic",self.displayCallback,10)
        self.frame_counter = 0
        self.path_param = './parameters/calibration_params.npz'
        self.mtx, self.dist = self.loadParam(self.path_param)

    def loadParam(self, path_param):
        # Load previously saved data
        with np.load(path_param) as data:
            mtx, dist = [data[i] for i in ('camera_matrix', 'dist_coeffs')]
        return mtx, dist
    
    def displayCallback(self,img_msg : Image):
        self.frame_counter +=1
        self.get_logger().info(f"The image number {self.frame_counter} has been read")
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        cv_image_und = self.undistortImage(cv_image)
        cv.imshow('Undistorted image',cv_image_und)
        cv.waitKey(1)

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

def main(args=None):
    rclpy.init(args=args)
    setup_node_test = SetupTest()
    rclpy.spin(setup_node_test)
    setup_node_test.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()