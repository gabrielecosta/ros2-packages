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
        # il client legge dal topic image_feed_topic
        self.subscription = self.create_subscription(Image, '/image_feed_topic', self.image_callback, 10)
        # creo quindi un client che si appoggia al servizio calibrate_camera
        # l'interfaccia del tipo di servizio è un trigger
        self.srv_client = self.create_client(Trigger, 'calibrate_camera')
        # definisco la cartella per le immagini da salvare per la calibrazione
        self.images_folder = 'images'
        os.makedirs(self.images_folder, exist_ok=True)
        self.image_count = 0
        # dovranno esserci almeno 10 immagini per avviare la calibrazione
        self.max_images = 10
        self.saved_images = []

    def image_callback(self, msg):
        '''
        Questa funzione legge il messaggio dal topic, poi se l'utente clicca 'q'
        allora salva l'immagine in una cartella (images) per la calibrazione.
        Una volta che l'utente ha scattato più di 10 mmagini (max_images),
        allora viene avviato il servizio di richiesta calibrazione
        '''
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
        # Invia una richiesta al server per la calibrazione
        if not self.srv_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Calibration service not available.')
            return
        # avvia un trigger di richiesta per effettuare la chiamata asincrona
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
