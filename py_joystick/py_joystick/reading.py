import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from py_joystick.joystickUtils import joyStickUtils

class ReadingJoystick(Node):

    def __init__(self):
        super().__init__("reading_joystick")
        self.subscription_ = self.create_subscription(String,"/controller_commands",self.readingCallback,10)
        self.joystick = joyStickUtils()

    def readingCallback(self, msg_data: String):
        data_dict = self.joystick.convertString(msg_data)
        self.get_logger().info(f"Commands: {data_dict}")
    
    def destroy_node(self):
        # routine che viene richiamata quando viene distrutto il nodo
        super().destroy_node()
    
    

def main(args=None):
    rclpy.init(args=args)
    readingnode = ReadingJoystick()
    rclpy.spin(readingnode)
    readingnode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()