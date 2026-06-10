#!/bin/bash

# 停止 Hunter SE bringup 后台进程

WS_DIR="/home/agilex03/r26-campus-autonomy-ws"
PID_FILE="$WS_DIR/.bringup_pids"

if [ ! -f "$PID_FILE" ]; then
    echo "未检测到运行中的 bringup（$PID_FILE 不存在）"
    exit 0
fi

# 读取 PID
read -r BRINGUP_PID PTP_PID < "$PID_FILE"

echo "停止 bringup (PID: $BRINGUP_PID)..."
kill "$BRINGUP_PID" 2>/dev/null

echo "停止 ptp4l (PID: $PTP_PID)..."
sudo kill "$PTP_PID" 2>/dev/null

# 等待进程退出
sleep 1

# 如果还在运行，强制终止
kill -9 "$BRINGUP_PID" 2>/dev/null
sudo kill -9 "$PTP_PID" 2>/dev/null

rm -f "$PID_FILE"
echo "Bringup 已停止。"
