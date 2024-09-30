import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from py_controller.nucleo_interface import NucleoController

from py_controller.joystickUtils import joyStickUtils

class ReadingJoystick(Node):

    def __init__(self):
        super().__init__("reading_joystick")
        self.Kx = 100
        self.Ky = 100
        self.Komega = 1000
        self.subscription_ = self.create_subscription(String,"/controller_commands",self.readingCallback,10)
        self.joystick = joyStickUtils()
        # self.timer_ = self.create_timer(1.0 / 30.0, self.publishCallback)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 20)
        self.servopublisher = self.create_publisher(JointState, "/servo_command", 20)

    def readingCallback(self, msg_data: String):
        data_dict = self.joystick.convertString(msg_data)
        self.get_logger().info(f"Commands: {data_dict}")
        # legge e pubblica il dizionario
        self.publishCallback(data_dict)

    def publishCallback(self, data_dict):
        x, y, omega, tl, tr = self.joystick.extract_vel_commands(data_dict)
        x = x * self.Kx
        y = y * self.Ky
        omega = omega * self.Komega
        tl_mapped = NucleoController.map_range(tl,-1, 1, 0, 180)
        tr_mapped = NucleoController.map_range(tr,-1, 1, 0, 180)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["servo_0", "servo_1"]
        # Assuming angles are in degrees, convert to radians
        joint_state.position = [tl_mapped, tr_mapped]
        msg = Twist()
        # comandi lineari
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = 0.0
        # comandi angolari
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(omega)

        self.servopublisher.publish(joint_state)
        self.publisher.publish(msg)
        self.get_logger().info(f"Commands: x={x}, y={y}, omega={omega}")

    
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