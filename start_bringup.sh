#!/bin/bash

# 启动 Hunter SE 底盘、雷达、URDF、点云转 LaserScan、PTP 时钟同步
# 无 IMU（使用底盘轮式里程计），无 RViz
# 启动后进程在后台运行，PID 保存到 .bringup_pids
# 停止：./stop_bringup.sh

WS_DIR="/home/agilex03/r26-campus-autonomy-ws"
PID_FILE="$WS_DIR/.bringup_pids"

# 如果已有 bringup 在运行，先停止
if [ -f "$PID_FILE" ]; then
    echo "检测到已有 bringup 运行，先停止..."
    bash "$WS_DIR/stop_bringup.sh"
    sleep 1
fi

# 初始化 CAN 总线
cd "$WS_DIR/src/ugv_sdk/scripts/" || exit
bash bringup_can2usb_500k.bash

# 加载 ROS 2 工作空间环境
cd "$WS_DIR" || exit
source install/setup.zsh

# 创建日志目录
mkdir -p "$WS_DIR/bringup_log"

# 后台启动 ROS 2 bringup
ros2 launch hunter_bringup hunter_bringup_no_imu.launch.py \
    > "$WS_DIR/bringup_log/bringup.log" 2>&1 &
BRINGUP_PID=$!

# 后台启动 PTP4L 时钟同步
sudo ptp4l -m -4 -i enp2s0 -S -l 5 \
    > "$WS_DIR/bringup_log/ptp4l.log" 2>&1 &
PTP_PID=$!

# 保存 PID
echo "$BRINGUP_PID $PTP_PID" > "$PID_FILE"

echo "========================================="
echo "  Hunter SE Bringup 已后台启动"
echo "========================================="
echo "  bringup PID: $BRINGUP_PID"
echo "  ptp4l   PID: $PTP_PID"
echo ""
echo "  查看日志："
echo "    tail -f $WS_DIR/bringup_log/bringup.log"
echo "    tail -f $WS_DIR/bringup_log/ptp4l.log"
echo ""
echo "  停止："
echo "    ./stop_bringup.sh"
echo "========================================="
