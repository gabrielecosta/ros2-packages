import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .udp_streaming import udpReceiver


class Receiver(Node):

    def __init__(self, udp_receiver):
        super().__init__("udp_receiver")
        self.cv_bridge = CvBridge()
        # creo un subscriber per ricevere la comunicazione dal topic camera_streaming
        self.subscription_ = self.create_subscription(String,"/camera_streaming",self.displayCallback,10)
        self.frame_counter = 0
        self.pub_frame = 0
        # creo un timer con la callback per mostrare il frame
        self.timer_ = self.create_timer(1.0 / 30.0, self.publishCallback)
        self.publisher_ = self.create_publisher(Image,"/image_feed_topic",20)
        self.udp_receiver = udp_receiver

    def publishCallback(self):
        self.pub_frame += 1
        frame = self.udp_receiver.receive()
        image_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        self.publisher_.publish(image_msg)
        self.get_logger().info(f"I published frame number {self.pub_frame}")
    
    def displayCallback(self, str_msg : String):
        '''
        Questa funzione riceve sia il messaggio dal topic che il frame ricevuto
        '''
        self.frame_counter +=1
        self.get_logger().info(f"The image number {self.frame_counter} has been read")
        # cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        # lettura del frame dal buffer
        frame = self.udp_receiver.receive()
        cv.imshow('Video_Feed',frame)
        cv.waitKey(1)

def main(args=None):
    # indirizzo ip del client di ricezione dello streaming
    udp_ip = "192.168.1.140"
    # udp_ip = "127.0.0.1"
    udp_port = 5005
    udp_receiver = udpReceiver(udp_ip, udp_port)
    rclpy.init(args=args)
    camera_node = Receiver(udp_receiver)
    rclpy.spin(camera_node)
    udp_receiver.closeConnection()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
