import rclpy
import cv2 as cv
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from py_camera_stream.udp_streaming import udpStreamer

class Streamer(Node):

    def __init__(self, udp_streamer):
        super().__init__("udp_streamer")
        # definisco l'indice della camera
        self.camera = cv.VideoCapture(0)
        # self.cv_bridge = CvBridge()
        # creo un timer per poter richiamare una funzione di callaback
        self.timer_ = self.create_timer(1.0 / 30.0, self.videoStreaming)
        self.frame_counter_ = 0
        # creo quindi un publisher su un topic per comunicare un messaggio di invio
        self.publisher_ = self.create_publisher(String,"/camera_streaming",20)
        # agente udp per lo streaming
        self.udp_streamer = udp_streamer

    def videoStreaming(self):
        # la callback viene effettuata ogni istante definita dal timer
        read_successful, frame = self.camera.read() # lettura dalla camera
        if read_successful:
            # image_msg = self.cv_bridge.cv2_to_imgmsg(frame)
            msg = String()
            msg.data = '[Frame #%d and sent via UDP]' % self.frame_counter_
            self.publisher_.publish(msg) # publish della stringa
            self.frame_counter_ += 1
            self.get_logger().info(f"I published frame number {self.frame_counter_}")
            # invio del frame tramite udp
            self.udp_streamer.send(frame)

    def destroy_node(self):
        # routine che viene richiamata quando viene distrutto il nodo
        self.camera.release()
        super().destroy_node()
    
    

def main(args=None):
    # definisco l'indirizzo ip del client che riceverà lo streaming
    udp_ip = "192.168.1.140"
    # udp_ip = "127.0.0.1"
    # definisco la porta abilitata al servizio udp di ricezione streaming
    udp_port = 5005
    # creo quindi l'agente udp passando come informazioni indirizzo ip e porta
    udp_streamer = udpStreamer(udp_ip, udp_port)
    rclpy.init(args=args)
    camera_publisher_node = Streamer(udp_streamer)
    rclpy.spin(camera_publisher_node)
    udp_streamer.closeConnection()
    camera_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()