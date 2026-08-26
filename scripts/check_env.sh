#!/usr/bin/env bash
# ===================================================================
# check_env.sh — 新電腦環境健檢
#
# 用途：到一台不確定裝了什麼的電腦（例如學校電腦）上，
#      跑這支腳本，立刻知道還缺哪些東西。
#
# 用法：  ./src/drone_control/scripts/check_env.sh
#
# 這支腳本「只讀不寫」，不會安裝或修改任何東西。
# ===================================================================

# 注意：這裡故意不用 set -e。
# 因為這支腳本的工作就是「找出壞掉的東西」，
# 任何一項檢查失敗都不該讓整支腳本中斷。

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

MISSING=0

ok()   { echo "  ✓ $1"; }
bad()  { echo "  ✗ $1"; echo "      → $2"; MISSING=$((MISSING + 1)); }
warn() { echo "  ! $1"; echo "      → $2"; }

echo "==================================================="
echo " 環境健檢"
echo " workspace : $WS_DIR"
echo " PX4       : $PX4_DIR"
echo "==================================================="

# --- 1. ROS 2 Humble ---------------------------------------------
echo
echo "[1/6] ROS 2 Humble"
if [ -f /opt/ros/humble/setup.bash ]; then
    ok "已安裝於 /opt/ros/humble"
else
    bad "找不到 /opt/ros/humble" \
        "見 https://docs.ros.org/en/humble/Installation.html"
fi

# --- 2. Gazebo Garden --------------------------------------------
# PX4 v1.14 的 gz_x500 需要 Garden（gz sim），不是 Classic（gazebo）。
echo
echo "[2/6] Gazebo Garden"
if command -v gz >/dev/null 2>&1; then
    GZ_VER="$(gz sim --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
    case "$GZ_VER" in
        7.*) ok "Garden $GZ_VER" ;;
        "")  warn "gz 存在但取不到版本" "手動確認：gz sim --version" ;;
        *)   warn "版本為 $GZ_VER，不是 Garden(7.x)" "PX4 v1.14 對應 Garden" ;;
    esac
elif command -v gazebo >/dev/null 2>&1; then
    bad "裝的是 Gazebo Classic，不是 Garden" \
        "兩者無法共存，裝 Garden 會移除 Classic：sudo apt install gz-garden"
else
    bad "沒有安裝 Gazebo" "sudo apt install gz-garden"
fi

# --- 3. Micro XRCE-DDS Agent -------------------------------------
echo
echo "[3/6] Micro XRCE-DDS Agent"
if command -v MicroXRCEAgent >/dev/null 2>&1; then
    ok "已安裝於 $(command -v MicroXRCEAgent)"
    # 用系統 Fast-DDS 建置的話，沒 source ROS 會找不到 .so
    if ! ldd "$(command -v MicroXRCEAgent)" 2>/dev/null | grep -q "not found"; then
        ok "動態函式庫都找得到"
    else
        warn "有函式庫找不到" "執行前先 source /opt/ros/humble/setup.bash"
    fi
else
    bad "找不到 MicroXRCEAgent" "見 README 的建置步驟（記得加 -DUAGENT_USE_SYSTEM_FASTDDS=ON）"
fi

# --- 4. PX4-Autopilot --------------------------------------------
echo
echo "[4/6] PX4-Autopilot"
if [ -d "$PX4_DIR" ]; then
    PX4_VER="$(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo '未知')"
    ok "原始碼存在（版本 $PX4_VER）"

    PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
    if [ -x "$PX4_BIN" ]; then
        ok "SITL 已編譯"
        # gz_bridge 沒編進去的話，Gazebo 永遠不會出現飛機
        if strings "$PX4_BIN" 2>/dev/null | grep -qx gz_bridge; then
            ok "gz_bridge 已包含"
        else
            bad "gz_bridge 沒被編進去" \
                "通常是編譯時找不到 gz-transport。重編：cd $PX4_DIR && make clean && make px4_sitl_default"
        fi
    else
        bad "SITL 尚未編譯" "cd $PX4_DIR && make px4_sitl_default（約 20~40 分鐘）"
    fi
else
    bad "找不到 $PX4_DIR" \
        "git clone -b v1.14.0 --recursive https://github.com/PX4/PX4-Autopilot.git $PX4_DIR"
fi

# --- 5. PX4 編譯需要的 Python 套件 --------------------------------
# 這幾個缺了會在編譯途中才爆，錯誤訊息還很難懂，所以先查。
echo
echo "[5/6] PX4 編譯用的 Python 套件"
# 注意：pyros-genmsg 這個「套件」匯入時的「模組」名稱是 genmsg，不是 pyros_genmsg
for mod in kconfiglib jinja2 em genmsg jsonschema; do
    if python3 -c "import $mod" >/dev/null 2>&1; then
        ok "$mod"
    else
        bad "$mod 缺少" "pip3 install --user kconfiglib jinja2 empy pyros-genmsg jsonschema"
    fi
done

# --- 6. workspace 本身 -------------------------------------------
echo
echo "[6/6] Workspace"
if [ -d "$WS_DIR/src/drone_control" ]; then
    ok "drone_control 存在"
else
    bad "找不到 src/drone_control" "確認你在 workspace 根目錄下執行"
fi

if [ -d "$WS_DIR/src/px4_msgs" ]; then
    PX4MSGS_REF="$(git -C "$WS_DIR/src/px4_msgs" describe --tags 2>/dev/null || echo '未知')"
    ok "px4_msgs 存在（$PX4MSGS_REF）"
else
    bad "px4_msgs 缺少（本來就不在版控內）" \
        "cd $WS_DIR && vcs import src < px4_deps.repos"
fi

if command -v vcs >/dev/null 2>&1; then
    ok "vcstool 可用"
else
    warn "沒有 vcstool" "還原 px4_msgs 需要：sudo apt install python3-vcstool"
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    ok "workspace 已編譯"
else
    warn "workspace 尚未編譯" "cd $WS_DIR && colcon build --symlink-install"
fi

# --- 總結 ---------------------------------------------------------
echo
echo "==================================================="
if [ "$MISSING" -eq 0 ]; then
    echo " 全部通過，可以開始飛"
else
    echo " 有 $MISSING 項缺少，請依上面的 → 指示處理"
fi
echo "==================================================="
exit "$MISSING"
