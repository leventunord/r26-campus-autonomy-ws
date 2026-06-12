import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

class YoloPersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_person_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('image_topic', '/image_stitched')
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.iou = self.get_parameter('iou_threshold').get_parameter_value().double_value
        
        self.get_logger().info(f'Loading model: {model_path} on {self.device}')
        self.model = YOLO(model_path, task='detect')
        
        self.bridge = CvBridge()
        
        # Subscription
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        
        # Added for optimization: frame counter
        self.frame_count = 0
        self.get_logger().info('YOLO Person Detector Node started')

    def image_callback(self, msg):
        self.frame_count += 1
        # Optimization 1: Frame skipping (process 1 out of 3 frames, reducing 30fps to ~10fps)
        if self.frame_count % 3 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # Inference - specifically class 0 for 'person' in COCO
        results = self.model.predict(
            source=cv_image,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            classes=[0],  # 0 is person in COCO
            imgsz=640,    # Optimization 2: Fixed inference resolution to drastically reduce computation
            verbose=False
        )
        
        # Detection Array message
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        # Results[0] is the result for the single image provided
        res = results[0]
        
        # Annotated image
        annotated_frame = res.plot()
        
        # Optimization 3: Resize annotated frame (to 960x480) before publishing to save DDS bandwidth
        annotated_frame = cv2.resize(annotated_frame, (960, 480))
        
        # Process detections
        for box in res.boxes:
            detection = Detection2D()
            detection.header = msg.header
            
            # Bounding box
            # box.xywh returns [x_center, y_center, width, height]
            xywh = box.xywh[0].tolist()
            detection.bbox.center.position.x = float(xywh[0])
            detection.bbox.center.position.y = float(xywh[1])
            detection.bbox.size_x = float(xywh[2])
            detection.bbox.size_y = float(xywh[3])
            
            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box.cls[0]))
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
            
        # Publish
        self.detection_pub.publish(detection_array)
        
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
