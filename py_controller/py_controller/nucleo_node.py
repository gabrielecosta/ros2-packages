#!/usr/bin/env python3

import math

import rclpy
import structlog
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

# Import the NucleoController class from nucleo_interface.py
# from py_controller.nucleo_interface import DummyNucleoController as NucleoController
from py_controller.nucleo_interface import NucleoController


class NucleoInterfaceNode(Node):
    def __init__(self):
        super().__init__("nucleo_interface_node")
        self.logger = structlog.get_logger()
        self.get_logger().info("Initializing Nucleo Interface Node")
        self.nucleo = NucleoController()

        # Subscribers
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.servosubscription = self.create_subscription(JointState, "/servo_command", self.servoCallback, 10)
        # Publishers
        self.angle_publisher = self.create_publisher(JointState, "/wheel_angles", 10)

        # Timer to read angles periodically
        #self.angle_timer = self.create_timer(0.1, self.publish_angles)

        # Services
        self.create_service(Trigger, "nucleo/stop_motors", self.stop_motors_callback)

        # Variables for robot state
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.5  # seconds

        # Start a timer to check for cmd_vel timeouts
        self.timeout_timer = self.create_timer(0.1, self.check_cmd_vel_timeout)

    def servoCallback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().error("Mismatch between joint names and positions.")
            return
        for joint_name, angle in zip(msg.name, msg.position):
            try:
                # Assuming joint_name is in the format 'servo_X' where X is the channel number
                channel = int(joint_name.split('_')[-1])
                self.nucleo.set_servo_angle(channel, angle)
                self.logger.info(f"Set servo {channel} to angle {angle} degrees.")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Invalid joint name format '{joint_name}': {e}")
            except Exception as e:
                self.get_logger().error(f"Failed to set servo {channel}: {e}")
        
    
    def cmd_vel_callback(self, msg: Twist):
        # Use the velocities directly without scaling
        x_dot = msg.linear.x  # m/s
        y_dot = msg.linear.y  # m/s
        theta_dot = msg.angular.z  # rad/s

        # Assuming that the NucleoController's set_inverse_kinematics method accepts floats
        # If it accepts integers, ensure to convert these to appropriate integer representations
        self.nucleo.set_inverse_kinematics(x_dot, y_dot, theta_dot)
        self.last_cmd_vel_time = self.get_clock().now()

    def publish_angles(self):
        try:
            angles = self.nucleo.read_angles()
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["joint1", "joint2", "joint3"]
            # Assuming angles are in degrees, convert to radians
            joint_state.position = [math.radians(angle) for angle in angles]
            self.angle_publisher.publish(joint_state)
        except IOError as e:
            self.get_logger().error(f"Error reading angles: {e}")

    def stop_motors_callback(self, request, response):
        self.nucleo.stop_all_steppers()
        response.success = True
        response.message = "All motors stopped."
        return response

    def check_cmd_vel_timeout(self):
        # Stop the robot if no cmd_vel messages have been received recently
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self.cmd_vel_timeout:
            # Stop the robot
            self.nucleo.set_inverse_kinematics(0, 0, 0)
            # Update the last command time to prevent repeated stops
            self.last_cmd_vel_time = current_time

    def destroy_node(self):
        # Override to ensure the NucleoController is closed
        self.get_logger().info("Shutting down Nucleo Interface Node")
        self.nucleo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NucleoInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
