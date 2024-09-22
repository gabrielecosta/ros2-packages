import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ExampleSubscriberNode(Node):

    def __init__(self):
        super().__init__("example_subscriber")
        self.cv_bridge = CvBridge()
        self.subscription_ = self.create_subscription(Image,"/image_feed_topic",self.displayCallback,10)
        self.frame_counter = 0

    def displayCallback(self,img_msg : Image):
        self.frame_counter +=1
        self.get_logger().info(f"The image number {self.frame_counter} has been read")
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        cv.imshow('Video_Feed_Subscriber',cv_image)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber_node = ExampleSubscriberNode()
    rclpy.spin(camera_subscriber_node)
    camera_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()