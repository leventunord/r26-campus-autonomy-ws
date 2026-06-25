#!/usr/bin/env python3
"""
七方位标定脚本 — 交互式采集 pixel_x ↔ true_yaw 对应关系

使用方法：
  1. 先启动 ./start_bringup.sh（确保相机和拼接节点在跑）
  2. 运行本脚本：
     source install/setup.bash
     python3 calibrate_directions.py
  3. 按照提示，让一个人依次站在 7 个方向
  4. 每个方向按 Enter 采集，脚本会自动检测人物并记录像素坐标
  5. 全部完成后，标定结果保存到 calibration_result.yaml
"""

import math
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    print("错误：请先安装 ultralytics: pip install ultralytics")
    sys.exit(1)


# ════════════════════ 配置区（可根据需要修改） ════════════════════

# 裁剪参数（必须与 yolo_node.py 中的 crop_left / crop_right 一致）
CROP_LEFT = 480
CROP_RIGHT = 480

# 原始全景图宽度
IMAGE_WIDTH = 3840

# YOLO 模型路径
MODEL_PATH = "yolo11n.pt"

# 7 个标定方向（弧度），从左到右排列
# 0° = 正前方，正值 = 左侧，负值 = 右侧
CALIBRATION_ANGLES_DEG = [-135, -90, -45, 0, 45, 90, 135]

# 每个方向采集多少帧取中位数（减少抖动）
SAMPLES_PER_DIRECTION = 5

# ════════════════════════════════════════════════════════════════


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.bridge = CvBridge()
        self.latest_image = None

        self.sub = self.create_subscription(
            Image,
            '/image_stitched',
            self.image_callback,
            10
        )
        self.get_logger().info('等待 /image_stitched 图像...')

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 异常: {e}')

    def grab_frame(self):
        """获取最新一帧图像，最多等待 3 秒"""
        self.latest_image = None
        start = time.time()
        while self.latest_image is None and (time.time() - start) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_image


def detect_person_x(model, image, crop_left, crop_right):
    """在裁剪后的图像上运行 YOLO，返回所有检测到人物的 (x_center_cropped, confidence) 列表"""
    h, w = image.shape[:2]
    x_start = crop_left
    x_end = w - crop_right
    cropped = image[:, x_start:x_end]

    results = model.predict(
        source=cropped,
        conf=0.4,
        iou=0.45,
        device='cpu',
        classes=[0],
        imgsz=640,
        verbose=False
    )

    detections = []
    for box in results[0].boxes:
        xywh = box.xywh[0].tolist()
        x_center_cropped = float(xywh[0])
        conf = float(box.conf[0])
        # 映射回原图坐标
        x_center_original = x_center_cropped + crop_left
        detections.append((x_center_original, conf))

    return detections


def main():
    rclpy.init()
    node = CalibrationNode()

    print("\n" + "=" * 60)
    print("  全景相机 7 方位标定工具")
    print("=" * 60)
    print(f"  裁剪参数: crop_left={CROP_LEFT}, crop_right={CROP_RIGHT}")
    print(f"  原图宽度: {IMAGE_WIDTH}")
    print(f"  每方向采集: {SAMPLES_PER_DIRECTION} 帧取中位数")
    print(f"  标定方向: {CALIBRATION_ANGLES_DEG}°")
    print("=" * 60)

    # 等待第一帧
    print("\n正在等待相机图像...")
    frame = node.grab_frame()
    if frame is None:
        print("错误：3 秒内未收到图像，请检查 /image_stitched 是否在发布")
        node.destroy_node()
        rclpy.shutdown()
        return

    actual_width = frame.shape[1]
    print(f"✓ 收到图像，实际宽度: {actual_width} px")
    if actual_width != IMAGE_WIDTH:
        print(f"  ⚠ 注意：实际宽度 {actual_width} 与配置 {IMAGE_WIDTH} 不同，将使用实际宽度")

    # 加载 YOLO 模型
    print(f"\n正在加载 YOLO 模型 ({MODEL_PATH})...")
    model = YOLO(MODEL_PATH, task='detect')
    print("✓ 模型加载完成\n")

    # 标定采集
    calibration_data = []  # [(pixel_x, true_yaw_rad)]

    for idx, angle_deg in enumerate(CALIBRATION_ANGLES_DEG):
        angle_rad = math.radians(angle_deg)

        # 方向描述
        if angle_deg == 0:
            direction_name = "正前方"
        elif angle_deg > 0:
            direction_name = f"左前方 {angle_deg}°"
        else:
            direction_name = f"右前方 {abs(angle_deg)}°"

        print("-" * 60)
        print(f"  [{idx + 1}/7] 请让人站在 {direction_name} (yaw = {angle_deg}°)")
        print(f"         确保视野内只有 1 个人")
        print("-" * 60)

        while True:
            input("  准备好后按 Enter 开始采集...")

            pixel_samples = []
            for s in range(SAMPLES_PER_DIRECTION):
                frame = node.grab_frame()
                if frame is None:
                    print(f"  ⚠ 第 {s+1} 帧获取失败，跳过")
                    continue

                detections = detect_person_x(model, frame, CROP_LEFT, CROP_RIGHT)

                if len(detections) == 0:
                    print(f"  ⚠ 第 {s+1} 帧未检测到人物")
                    continue
                elif len(detections) > 1:
                    # 取置信度最高的
                    detections.sort(key=lambda d: d[1], reverse=True)
                    print(f"  ⚠ 第 {s+1} 帧检测到 {len(detections)} 人，取置信度最高的")

                pixel_samples.append(detections[0][0])
                print(f"  ✓ 第 {s+1} 帧: pixel_x = {detections[0][0]:.1f} (conf={detections[0][1]:.2f})")

            if len(pixel_samples) < 3:
                print(f"\n  ✗ 有效帧数不足 ({len(pixel_samples)}/{SAMPLES_PER_DIRECTION})，请重试")
                continue

            # 取中位数
            pixel_samples.sort()
            median_pixel = pixel_samples[len(pixel_samples) // 2]
            print(f"\n  ★ {direction_name}: pixel_x 中位数 = {median_pixel:.1f}")

            # 显示当前线性公式计算出的 yaw（用于对比）
            linear_yaw = -((median_pixel - (actual_width / 2.0)) / float(actual_width)) * 2.0 * math.pi
            linear_yaw_deg = math.degrees(linear_yaw)
            print(f"    （线性公式得到: {linear_yaw_deg:.1f}°，真实: {angle_deg}°，偏差: {linear_yaw_deg - angle_deg:.1f}°）")

            confirm = input("  确认采用此数据？(Y/n): ").strip().lower()
            if confirm in ('', 'y', 'yes'):
                calibration_data.append({
                    'pixel_x': round(median_pixel, 1),
                    'true_yaw_deg': angle_deg,
                    'true_yaw_rad': round(angle_rad, 6),
                    'linear_yaw_deg': round(linear_yaw_deg, 1),
                    'error_deg': round(linear_yaw_deg - angle_deg, 1),
                })
                break
            else:
                print("  重新采集此方向...\n")

    # 输出结果
    print("\n" + "=" * 60)
    print("  标定完成！结果汇总：")
    print("=" * 60)
    print(f"  {'方向':>8s}  {'pixel_x':>8s}  {'线性yaw':>8s}  {'偏差':>6s}")
    print("-" * 40)
    for d in calibration_data:
        print(f"  {d['true_yaw_deg']:>7.0f}°  {d['pixel_x']:>8.1f}  {d['linear_yaw_deg']:>7.1f}°  {d['error_deg']:>+5.1f}°")

    # 保存到文件
    output = {
        'image_width': actual_width,
        'crop_left': CROP_LEFT,
        'crop_right': CROP_RIGHT,
        'calibration_points': calibration_data,
    }

    output_path = 'calibration_result.yaml'
    with open(output_path, 'w') as f:
        yaml.dump(output, f, default_flow_style=False, allow_unicode=True)

    print(f"\n  ✓ 标定结果已保存到: {output_path}")
    print("  下一步：将标定数据应用到 yolo_node.py 的分段线性插值中")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
