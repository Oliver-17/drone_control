#!/usr/bin/env bash
# ===================================================================
# record_flight.sh — 錄下一次飛行的所有相關 topic
#
# 用途：ros2 bag 把原始資料完整錄下來，事後可以重播、畫圖、比對。
#      文字 log 只有你 RCLCPP_INFO 印的東西；bag 錄的是每一筆訊息。
#
# 用法：
#   ./record_flight.sh            # 無 namespace（單機、ros2 run）
#   ./record_flight.sh MAV1       # 有 namespace
#   ./record_flight.sh MAV1 MAV2 MAV3   # 三機一起錄
#
# 在飛之前先啟動，Ctrl+C 結束錄製。
# ===================================================================
set -u

OUT_ROOT="${FLIGHT_LOG_DIR:-$HOME/flight_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUT_ROOT/$STAMP"

# 每台飛機要錄的 topic。
#   /fmu/out/* 是飛控回報的狀態（飛機實際怎麼動）
#   /fmu/in/*  是我們送出的指令（我們叫它怎麼動）
# 兩邊都錄才能事後比對「指令 vs 實際」，這是除錯的關鍵。
SUFFIXES=(
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_status"
    "/fmu/out/vehicle_attitude"
    "/fmu/in/offboard_control_mode"
    "/fmu/in/trajectory_setpoint"
    "/fmu/in/vehicle_command"
)

# 組出完整 topic 清單。沒給參數時用空前綴（對應無 namespace 的情況）。
NAMESPACES=("$@")
if [ ${#NAMESPACES[@]} -eq 0 ]; then
    NAMESPACES=("")
fi

TOPICS=()
for ns in "${NAMESPACES[@]}"; do
    prefix=""
    [ -n "$ns" ] && prefix="/$ns"
    for s in "${SUFFIXES[@]}"; do
        TOPICS+=("${prefix}${s}")
    done
done

echo "==================================================="
echo " 錄製飛行資料"
echo " 輸出：$OUT_DIR"
echo " topic 數：${#TOPICS[@]}"
for t in "${TOPICS[@]}"; do echo "   $t"; done
echo "==================================================="
echo " Ctrl+C 結束錄製"
echo

mkdir -p "$OUT_ROOT"

# --max-cache-size 加大可減少高頻資料的掉包。
# 注意：即使 topic 還沒有人發布，ros2 bag 也會等待並在出現時開始錄。
ros2 bag record -o "$OUT_DIR" --max-cache-size 10485760 "${TOPICS[@]}"

echo
echo "==================================================="
echo " 錄製完成：$OUT_DIR"
echo
echo " 查看內容：  ros2 bag info $OUT_DIR"
echo " 重新播放：  ros2 bag play $OUT_DIR"
echo "==================================================="
