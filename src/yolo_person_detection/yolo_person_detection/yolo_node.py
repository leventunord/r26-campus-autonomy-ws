import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import math
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
        self.declare_parameter('image_width', 3840)
        self.declare_parameter('crop_left', 480)   # 左侧裁掉的像素数（对应后方左半）
        self.declare_parameter('crop_right', 480)  # 右侧裁掉的像素数（对应后方右半）
        self.declare_parameter('frame_id', 'base_link')
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.iou = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.img_width = self.get_parameter('image_width').value
        self.crop_left = self.get_parameter('crop_left').value
        self.crop_right = self.get_parameter('crop_right').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Calibration lookup table: (pixel_x, yaw_rad)
        # Sorted by pixel_x ascending (right side → left side of image)
        # Data from calibrate_directions.py
        self.calib_table = [
            (535.6, 2.356194),    # 135° (left rear)
            (895.2, 1.570796),    #  90° (left)
            (1310.9, 0.785398),   #  45° (left front)
            (1797.5, 0.0),        #   0° (front)
            (2394.4, -0.785398),  # -45° (right front)
            (2667.8, -1.570796),  # -90° (right)
            (2989.2, -2.356194),  # -135° (right rear)
        ]
        
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
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/person_directions', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)
        
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

        # Crop: remove left and right edges (rear view) to avoid duplicate detections
        h, w = cv_image.shape[:2]
        x_start = self.crop_left
        x_end = w - self.crop_right
        if x_start >= x_end:
            self.get_logger().error('crop_left + crop_right >= image width, skipping')
            return
        cv_image_cropped = cv_image[:, x_start:x_end]

        # Inference on cropped image
        results = self.model.predict(
            source=cv_image_cropped,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            classes=[0],  # 0 is person in COCO
            imgsz=640,
            verbose=False
        )
        
        # Detection Array message
        # Results[0] is the result for the single image provided
        res = results[0]
        
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'person_direction'
        marker_array.markers.append(delete_marker)
        
        # Red reference arrow: always points to robot's front (yaw=0)
        front_marker = Marker()
        front_marker.header.frame_id = self.frame_id
        front_marker.header.stamp = msg.header.stamp
        front_marker.ns = 'robot_front'
        front_marker.id = 0
        front_marker.type = Marker.ARROW
        front_marker.action = Marker.ADD
        front_marker.pose.position.z = 0.3
        front_marker.pose.orientation.x = 0.0
        front_marker.pose.orientation.y = 0.0
        front_marker.pose.orientation.z = 0.0
        front_marker.pose.orientation.w = 1.0
        front_marker.scale.x = 1.5
        front_marker.scale.y = 0.12
        front_marker.scale.z = 0.12
        front_marker.color.a = 0.9
        front_marker.color.r = 1.0
        front_marker.color.g = 0.0
        front_marker.color.b = 0.0
        marker_array.markers.append(front_marker)
        
        # Process detections directly into markers
        for i, box in enumerate(res.boxes):
            # Bounding box (coordinates are in the cropped image)
            xywh = box.xywh[0].tolist()
            x_center_cropped = float(xywh[0])
            
            # Map back to original full-image pixel coordinate
            x_center = x_center_cropped + self.crop_left
            
            # Calculate Yaw using calibrated piecewise linear interpolation
            yaw = self._pixel_to_yaw(x_center)
            
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = msg.header.stamp
            marker.ns = 'person_direction'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.z = 0.3
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x = 1.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.8
            marker.color.r = 0.0
            marker.color.g = 1.0 # Green
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

        # Publish annotated image with bounding boxes
        annotated = res.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

    def _pixel_to_yaw(self, pixel_x: float) -> float:
        """Piecewise linear interpolation from pixel_x to yaw using calibration table."""
        table = self.calib_table  # sorted by pixel_x ascending
        
        # Clamp / extrapolate beyond calibration range
        if pixel_x <= table[0][0]:
            # Extrapolate beyond leftmost calibration point
            px0, yaw0 = table[0]
            px1, yaw1 = table[1]
            slope = (yaw1 - yaw0) / (px1 - px0)
            return yaw0 + slope * (pixel_x - px0)
        
        if pixel_x >= table[-1][0]:
            # Extrapolate beyond rightmost calibration point
            px0, yaw0 = table[-2]
            px1, yaw1 = table[-1]
            slope = (yaw1 - yaw0) / (px1 - px0)
            return yaw1 + slope * (pixel_x - px1)
        
        # Find the two surrounding calibration points and interpolate
        for j in range(len(table) - 1):
            px0, yaw0 = table[j]
            px1, yaw1 = table[j + 1]
            if px0 <= pixel_x <= px1:
                t = (pixel_x - px0) / (px1 - px0)
                return yaw0 + t * (yaw1 - yaw0)
        
        # Fallback (should never reach here)
        return 0.0

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
