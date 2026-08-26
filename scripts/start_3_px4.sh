#!/usr/bin/env bash
# =============================================================================
#  start_3_px4.sh — 啟動三台 PX4 SITL（MAV1 / MAV2 / MAV3）在同一個 Gazebo 世界
#
#  用法：
#      ./start_3_px4.sh            # 有 Gazebo 視窗
#      HEADLESS=1 ./start_3_px4.sh # 無視窗（省資源；需要看畫面時另開終端跑 gz sim -g）
#
#  重要：等腳本印出「三台全部就緒」才可以去跑 ros2 launch。
#        提早跑的話，先連上的那台會自己起飛，另外兩台還在等 PX4，
#        三台的動作就會完全錯開。
#
#  停止：
#      pkill -x px4 ; pkill -f "gz sim"
#      （第二行一定要用 -f：gz 是 Ruby 包裝腳本，程序名是 ruby，-x 抓不到）
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

if [ ! -x "$BUILD_DIR/bin/px4" ]; then
    echo "找不到 $BUILD_DIR/bin/px4，請先執行： cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

# 載入 Gazebo 的模型/世界搜尋路徑（PX4 編譯時自動產生），否則 gz 找不到 x500 模型
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"

NAMES=("MAV1" "MAV2" "MAV3")
POSES=("0,0"  "0,3"  "0,-3")   # NED：第一個是北，第二個是東。橫向間隔 3 公尺

# --- 小工具：輪詢等待某個條件成立 -------------------------------------------
# 用「等到真的好了」取代「睡固定秒數」。固定 sleep 在機器慢的時候會不夠、
# 快的時候又浪費時間，而且永遠不知道到底成功了沒。
wait_for() {
    local desc="$1" timeout="$2"; shift 2
    local waited=0
    while ! "$@" >/dev/null 2>&1; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$timeout" ]; then
            echo "  ✗ 逾時（${timeout}s）：$desc"
            return 1
        fi
    done
    echo "  ✓ $desc（耗時 ${waited}s）"
    return 0
}

world_is_up()      { gz topic -l 2>/dev/null | grep -qE "/world/.*/clock"; }
instance_is_ready() { grep -q "uxrce_dds_client.*vehicle_local_position" "$1"; }

# --- 清理舊程序 --------------------------------------------------------------
echo "清掉可能殘留的舊程序…"
pkill -x px4 || true
pkill -f "gz sim" || true      # 注意是 -f，不是 -x
sleep 2

# --- 依序啟動三台 ------------------------------------------------------------
for i in 0 1 2; do
    NAME="${NAMES[$i]}"
    POSE="${POSES[$i]}"
    WORK_DIR="$BUILD_DIR/instance_$i"

    mkdir -p "$WORK_DIR"
    rm -f "$WORK_DIR/out.log"     # 清掉舊 log，避免輪詢時誤判成已就緒

    echo "啟動 $NAME  (instance $i, MAV_SYS_ID $((i+1)), 位置 N,E = $POSE)"

    (
        cd "$WORK_DIR"
        # PX4_UXRCE_DDS_NS  → topic 前綴變成 /MAV1/fmu/...（rcS:273-277）
        # PX4_SYS_AUTOSTART → 4001 = gz_x500 機體
        # PX4_GZ_MODEL_POSE → 世界裡的 spawn 位置，三台才不會疊在一起
        # -i $i             → instance 編號，決定 MAV_SYS_ID 與 UXRCE_DDS_KEY（rcS:131-132）
        PX4_UXRCE_DDS_NS="$NAME" \
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL=x500 \
        PX4_GZ_MODEL_POSE="$POSE" \
        HEADLESS="${HEADLESS:-}" \
        "$BUILD_DIR/bin/px4" -i "$i" -d "$BUILD_DIR/etc" \
            > "$WORK_DIR/out.log" 2>&1 &
    )

    # 第一台負責建立 Gazebo 世界，後兩台會偵測到世界已存在而直接加入
    # （px4-rc.simulator 的 gz_world 判斷）。所以第一台一定要先等世界起來，
    # 否則後兩台可能各自再開一個世界。
    if [ "$i" -eq 0 ]; then
        wait_for "Gazebo 世界已建立" 90 world_is_up
    fi

    # 等這台真的把 topic 建好，再啟動下一台
    wait_for "$NAME 已連上 XRCE Agent" 90 instance_is_ready "$WORK_DIR/out.log"
done

echo
echo "=================================================="
echo " 三台全部就緒 — 現在可以去終端 3 跑："
echo "   ros2 launch drone_control three_drones.launch.py"
echo "=================================================="
echo
echo "log 位置："
for i in 0 1 2; do echo "  ${NAMES[$i]}: $BUILD_DIR/instance_$i/out.log"; done
echo
echo "停止： pkill -x px4 ; pkill -f 'gz sim'"
