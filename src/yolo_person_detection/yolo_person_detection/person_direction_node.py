import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
import math

class PersonDirectionNode(Node):
    def __init__(self):
        super().__init__('person_direction_node')
        
        self.declare_parameter('image_width', 3840)
        self.declare_parameter('yaw_offset', 0.0) # 但这里的参数应该在launch中修改。
        self.declare_parameter('frame_id', 'base_link')
        
        self.img_width = self.get_parameter('image_width').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/person_directions', 10)
        
        # 增加一个定时器，每 0.1 秒发布一次红箭头，保证没人时也能看到正前方
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Person Direction Node started. Red arrow = Robot Front.')

    def timer_callback(self):
        # 这个定时器只负责发布红色的“正前方”参考箭头
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_front_marker())
        self.marker_pub.publish(marker_array)

    def create_front_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_front'
        marker.id = 999
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0 # 始终指向坐标系的 X 正方向
        marker.scale.x = 1.0
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 1.0 # 红色
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def detection_callback(self, msg):
        # 当有检测结果时，发布绿色的“人物方向”箭头
        marker_array = MarkerArray()
        
        # 清除上一帧的人物箭头 (只清除 person_direction 命名空间下的)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'person_direction'
        marker_array.markers.append(delete_marker)
        
        # 顺便把红箭头也带上
        marker_array.markers.append(self.create_front_marker())
        
        for i, detection in enumerate(msg.detections):
            x_center = detection.bbox.center.position.x
            
            # 计算 Yaw (弧度)
            # 这里的逻辑：全景图宽度 img_width 对应 2*PI
            # (x_center - img_width/2) / img_width 是偏离中心的百分比
            yaw = - ((x_center - (self.img_width / 2.0)) / float(self.img_width)) * 2.0 * math.pi
            # 加上偏移量
            yaw += self.yaw_offset
            
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            
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
            marker.color.g = 1.0 # 绿色
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
