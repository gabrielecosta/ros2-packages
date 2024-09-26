import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")
        #loading models and naming classes
        self.model = YOLO("../resource/yolov10n.pt") # in questo caso ho già scaricato i pesi
        self.classesnames = ["vase", "pottedplant"]
        self.class_indexes = [75, 58]
        #initializing ros2 opencv tools
        self.cv_bridge = CvBridge()
        self.subscription_ = self.create_subscription(Image,"/image_feed_topic",self.detectAndVisualizeCallback,10)
        

    @staticmethod
    def predict(chosen_model, img, classes=[], conf=0.5):
        if classes:
           results = chosen_model.predict(img, classes=classes, conf=conf)
        else:
           results = chosen_model.predict(img, conf=conf)

        return results
    
    @staticmethod
    def predict_and_detect(chosen_model, img, classes=[], conf=0.5, rectangle_thickness=2, text_thickness=1):
        results = DetectorNode.predict(chosen_model, img, classes, conf=conf)
        for result in results:
            # Process results list
            boxes = result.boxes  # Boxes object for bounding box outputs
            print(f'Boxes: {boxes}')
            masks = result.masks  # Masks object for segmentation masks outputs
            print(f'Masks: {masks}')
            keypoints = result.keypoints  # Keypoints object for pose outputs
            print(f'Keypoints: {keypoints}')
            probs = result.probs  # Probs object for classification outputs
            print(f'Probs: {probs}')
            obb = result.obb  # Oriented boxes object for OBB outputs
            print(f'OBB: {obb}')
            for box in result.boxes:
                cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), rectangle_thickness)
                cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), text_thickness)
        return img, results


    def detectAndVisualizeCallback(self,img_msg : Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        # read the image
        result_img, _ = DetectorNode.predict_and_detect(self.model, cv_image, classes=self.class_indexes, conf=0.5) #missing dete
        # Display the frame with the detected objects
        cv2.imshow("Live Inference", result_img)
        cv2.waitKey(1)
        
        

def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()