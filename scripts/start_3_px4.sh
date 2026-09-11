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

# --- 強制 Gazebo 走 NVIDIA 獨顯 -----------------------------------------------
# 這台筆電是雙顯卡（NVIDIA + AMD 內顯），driver 設定是 PRIME "on-demand"。
# on-demand 的意思是「預設一律用內顯，除非程式自己要求 offload」——
# 所以不加下面這兩個變數的話，Gazebo 會整場跑在內顯上。
# 三機同場時內顯畫不動 → 物理步進被拖慢 → SITL 是 lockstep，PX4 收不到 IMU
# → `Accel #0 fail: TIMEOUT` → EKF 收斂不了 → 預檢一路失敗。
# 環境變數會被子程序繼承，所以在這裡 export 一次，PX4 起的 gz sim 就吃得到。
#
# ⚠️ 這裡要問的是「獨顯現在能不能用」，不是「nvidia-smi 這個檔案在不在」。
# 2026-09-11 踩到：apt 把驅動從 595.84 升到 595.91 但沒重開機，記憶體裡的
# 核心模組還是舊版 → nvidia-smi 檔案還在、但離開碼 18（NVML 版本不符）。
# 舊的 `command -v` 守門條件照樣成立，於是強制走一條壞掉的路 →
# Qt 建不出 OpenGL context → GUI 秒退 → Gazebo 看起來「自己關掉」，
# 而錯誤只寫進 ~/.gz/auto_default.log，終端上一個字都看不到，極難查。
if nvidia-smi -L >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only   # Gazebo 若走 Vulkan 後端才會用到
    echo "已啟用 NVIDIA offload（用 nvidia-smi 可確認 gz sim 有出現在程序列表）"
elif command -v nvidia-smi >/dev/null 2>&1; then
    # nvidia-smi 在，但跑不動 —— 獨顯現在不能用，硬推會讓 Gazebo 默默死掉，
    # 所以退回內顯並把原因印在終端上（詳見上面那段註解）。
    echo "⚠️  nvidia-smi 跑不起來，獨顯現在無法使用 —— 改走內顯。原因："
    nvidia-smi -L 2>&1 | sed 's/^/     /'
    echo "     最常見的是「驅動升級後還沒重開機」（核心模組還是舊版），重開機就會好。"
    echo "     內顯畫多機會拖慢物理步進 → 可能 Accel TIMEOUT → 預檢失敗，先用 DRONES=1 比較保險。"
else
    echo "找不到 nvidia-smi，維持預設顯示卡"
fi

# --- 把 gz-transport 綁在回環位址 ------------------------------------------
# 不設這個的話，gz-transport 會綁到「所有」網路介面，IMU 訊息的傳遞會出現
# 延遲抖動。SITL 是 lockstep，感測器資料一遲到就會 `Accel #0 fail: TIMEOUT`，
# 接著 EKF 的姿態與高度估計跟著劣化，起飛後觸發失效保護 RTL，最後翻覆墜毀。
#
# 為什麼 `make px4_sitl gz_x500` 不會有這個問題：那條路徑是
# `cmake -E env PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 bin/px4`，
# cmake 幫忙包了這個變數；本腳本直接呼叫 bin/px4，就繞過了那一層。
#
# 實測 2026-08-31：加這一行前 Accel TIMEOUT 112~127 次、三台全部墜毀；
#                  加之後 0 次、穩定懸停降落。
export GZ_IP=127.0.0.1

# 載入 Gazebo 的模型/世界搜尋路徑（PX4 編譯時自動產生），否則 gz 找不到 x500 模型
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"

NAMES=("MAV1" "MAV2" "MAV3")
POSES=("0,0"  "0,3"  "0,-3")   # ENU：第一個是東、第二個是北，間隔 3 公尺（沿南北排開）。
                               # 為什麼不是 NED：px4-rc.gzsim:116-130 把這串原封不動
                               # 塞進 SDF 的 <pose>，中間沒有座標轉換，而 SDF 世界是 ENU
                               #（world 檔裡的 <world_frame_orientation>ENU</...>）

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

# --- 補上 v1.17 SITL 的預檢參數 ----------------------------------------------
# v1.17 的機型檔 4001_gz_x500:51 多了 `param set-default NAV_DLL_ACT 2`
# （v1.14 沒有這行），沒開 QGC 就會 `Preflight Fail: No connection to the GCS`，
# ARM 永遠不會成功。CBRK_SUPPLY_CHK 則是關掉 SITL 沒有的電源檢查。
#
# 為什麼不用 v1.17 新增的 PX4_PARAM_xxx 環境變數：那個在 rcS:129 執行，
# 機型檔卻在 rcS:233 才被 source，`param set-default` 會蓋掉它 ——
# CBRK_SUPPLY_CHK 有效（機型檔沒設），NAV_DLL_ACT 無效。實測過。
#
# 為什麼要逐台設：每台跑在自己的 instance_$i 工作目錄，參數檔是
# instance_$i/parameters.bson，跟單機用的 rootfs/parameters.bson 是不同檔案。
# px4-param 是 PX4 的 client 執行檔，--instance 可指定對哪一台下指令
# （platforms/posix/src/px4/common/main.cpp:154），不必去搶 console。
fix_preflight_params() {
    local i="$1"
    local param="$BUILD_DIR/bin/px4-param"
    "$param" --instance "$i" set NAV_DLL_ACT 0        >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set CBRK_SUPPLY_CHK 894281 >/dev/null 2>&1 || return 1
    # save 之後會寫進 instance_$i/parameters.bson，下次重跑就不用再設
    "$param" --instance "$i" save                     >/dev/null 2>&1 || return 1
    return 0
}

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

    if fix_preflight_params "$i"; then
        echo "  ✓ $NAME 預檢參數已設定（NAV_DLL_ACT=0, CBRK_SUPPLY_CHK）"
    else
        echo "  ⚠ $NAME 預檢參數設定失敗 —— ARM 可能會被擋，手動確認："
        echo "      $BUILD_DIR/bin/px4-param --instance $i show NAV_DLL_ACT"
    fi
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
