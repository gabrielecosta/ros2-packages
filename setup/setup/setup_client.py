import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import cv2
from cv_bridge import CvBridge
import os

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_feed_topic', self.image_callback, 10)
        self.srv_client = self.create_client(Trigger, 'calibrate_camera')
        self.images_folder = 'images'
        os.makedirs(self.images_folder, exist_ok=True)
        self.image_count = 0
        self.max_images = 10
        self.saved_images = []

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('Camera Feed', image)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            image_path = os.path.join(self.images_folder, f'image_{self.image_count}.png')
            cv2.imwrite(image_path, image)
            self.get_logger().info(f'Saved image: {image_path}')
            self.saved_images.append(image_path)
            self.image_count += 1

            if self.image_count >= self.max_images:
                self.get_logger().info('Collected enough images, requesting calibration...')
                self.request_calibration()

    def request_calibration(self):
        # Send a request to the server for calibration
        if not self.srv_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Calibration service not available.')
            return
        
        request = Trigger.Request()
        future = self.srv_client.call_async(request)
        future.add_done_callback(self.calibration_response)

    def calibration_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Calibration completed successfully!')
            else:
                self.get_logger().info('Calibration failed.')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
