#!/bin/bash

# 停止 Hunter SE bringup 后台进程 - 精准清理版

WS_DIR="/home/agilex03/r26-campus-autonomy-ws"
PID_FILE="$WS_DIR/.bringup_pids"

echo "========================================="
echo "  正在停止 Hunter SE 系统..."
echo "========================================="

# 1. 首先尝试通过 PID 优雅停止
if [ -f "$PID_FILE" ]; then
    read -r BRINGUP_PID PTP_PID < "$PID_FILE"
    
    echo "-> 停止 launch 进程 (PID: $BRINGUP_PID)..."
    kill "$BRINGUP_PID" 2>/dev/null
    
    echo "-> 停止 PTP 同步进程 (PID: $PTP_PID)..."
    sudo kill "$PTP_PID" 2>/dev/null
    
    sleep 1
    rm -f "$PID_FILE"
fi

# 2. 精准清理：只杀掉本项目特有的执行文件名
# 这样即使 launch 挂了，这些特定的子进程也不会变成“孤儿”霸占资源
echo "-> 检查并清理残留的感知与驱动节点..."
SPECIFIC_NODES=("usb_cam_node_exe" "yolo_node" "fisheye_stitch" "hesai_ros_driver" "hunter_base_node")
for node in "${SPECIFIC_NODES[@]}"; do
    pkill -9 -f "$node" 2>/dev/null
done

# 3. 强制释放摄像头资源（这是解决“设备忙”的核心）
if fuser /dev/video0 >/dev/null 2>&1; then
    echo "-> 释放 /dev/video0 硬件占用..."
    fuser -k -9 /dev/video0 2>/dev/null
fi

echo "========================================="
echo "  精准清理完成。"
echo "========================================="
