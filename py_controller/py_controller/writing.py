import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from py_controller.joystickUtils import joyStickUtils

class WritingJoystick(Node):

    def __init__(self):
        super().__init__("writing_joystick")
        self.timer_ = self.create_timer(0.1, self.inputCallback)
        self.frame_counter_ = 0
        self.publisher_ = self.create_publisher(String,"/controller_commands",20)
        self.joystick = joyStickUtils()

    def inputCallback(self):
        data_controller = self.joystick.controllerReading()
        msg = String()
        if data_controller:
            msg.data = self.joystick.convertJson(data_controller)
        else:
            msg.data = ''
        self.publisher_.publish(msg) # publish della stringa
        self.get_logger().info(f"Commands: {data_controller}")
    
    def destroy_node(self):
        # routine che viene richiamata quando viene distrutto il nodo
        super().destroy_node()
    
    

def main(args=None):
    rclpy.init(args=args)
    writingnode = WritingJoystick()
    rclpy.spin(writingnode)
    writingnode.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()