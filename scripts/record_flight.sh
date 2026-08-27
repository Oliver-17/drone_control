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
# 版本後綴會自動偵測（PX4 v1.16+ 的 /fmu/out/ 是 …_v1，v1.14 沒有）。
# 要手動指定就設環境變數：
#   TOPIC_SUFFIX=_v1 ./record_flight.sh MAV1
#   TOPIC_SUFFIX=''  ./record_flight.sh          # 強制不加後綴
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
# out 需要版本後綴，in 不需要 —— 實測 PX4 v1.17.0：
#   /MAV1/fmu/out/vehicle_local_position_v1   有 _v1
#   /MAV1/fmu/in/trajectory_setpoint          沒有
OUT_TOPICS=(
    "/fmu/out/vehicle_local_position"
    "/fmu/out/vehicle_status"
    "/fmu/out/vehicle_attitude"
)
IN_TOPICS=(
    "/fmu/in/offboard_control_mode"
    "/fmu/in/trajectory_setpoint"
    "/fmu/in/vehicle_command"
)

# 組出完整 topic 清單。沒給參數時用空前綴（對應無 namespace 的情況）。
NAMESPACES=("$@")
if [ ${#NAMESPACES[@]} -eq 0 ]; then
    NAMESPACES=("")
fi

# 先把現有的 topic 抓下來，後面拿來自動偵測後綴、以及警告錄不到的 topic。
# 抓一次就好 —— ros2 topic list 每次要等 DDS 探索，很慢。
LIVE_TOPICS="$(ros2 topic list 2>/dev/null || true)"

# 自動偵測版本後綴。
# 為什麼要自動：忘記加 _v1 的話 ros2 bag 不會報錯，只會安靜地錄到空檔案，
# 等你事後想看資料才發現什麼都沒有 —— 那時飛行已經結束，補不回來。
FIRST_NS="${NAMESPACES[0]}"
FIRST_PREFIX=""
[ -n "$FIRST_NS" ] && FIRST_PREFIX="/$FIRST_NS"

if [ -z "${TOPIC_SUFFIX+x}" ]; then
    if echo "$LIVE_TOPICS" | grep -qx "${FIRST_PREFIX}/fmu/out/vehicle_local_position_v1"; then
        TOPIC_SUFFIX="_v1"
        echo "偵測到 PX4 v1.16+ 的版本化 topic，後綴使用 '_v1'"
    else
        TOPIC_SUFFIX=""
    fi
else
    echo "使用指定的後綴：'${TOPIC_SUFFIX}'"
fi

TOPICS=()
for ns in "${NAMESPACES[@]}"; do
    prefix=""
    [ -n "$ns" ] && prefix="/$ns"
    for t in "${OUT_TOPICS[@]}"; do
        TOPICS+=("${prefix}${t}${TOPIC_SUFFIX}")
    done
    for t in "${IN_TOPICS[@]}"; do
        TOPICS+=("${prefix}${t}")
    done
done

echo "==================================================="
echo " 錄製飛行資料"
echo " 輸出：$OUT_DIR"
echo " topic 數：${#TOPICS[@]}"
# 對照現有 topic 標記。打勾的才真的錄得到，沒打勾的會錄成空檔案。
MISSING=0
for t in "${TOPICS[@]}"; do
    if echo "$LIVE_TOPICS" | grep -qx "$t"; then
        echo "   ✓ $t"
    else
        echo "   ✗ $t   （目前不存在）"
        MISSING=$((MISSING + 1))
    fi
done
echo "==================================================="
if [ "$MISSING" -gt 0 ]; then
    echo " ⚠️  有 $MISSING 個 topic 現在不存在。"
    echo "     若飛控還沒開機屬正常；否則請先用 ros2 topic list 核對名稱。"
    echo "==================================================="
fi
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
