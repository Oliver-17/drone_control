# drone_control — PX4 Offboard 控制節點

ROS 2 Humble + PX4 SITL 的 offboard 控制 package，目標是三機編隊。
目前進度：單機 / 三機起飛、懸停、降落已完成並實測驗證。

> 這個 repo **本身就是一個 ROS 2 package**，不是 workspace。
> 把它 clone 進你既有 workspace 的 `src/` 底下即可（見下方安裝）。

---

## 環境版本（已驗證可用的組合）

| 項目 | 版本 |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4-Autopilot | v1.14.0 |
| px4_msgs | `release/1.14` @ `ffb6e80` |
| Micro XRCE-DDS Agent | v2.4.2 |
| Gazebo | **Garden 7.9.0**（不是 Classic 11） |

> Gazebo Garden 與 Classic **無法共存**：`gz-tools2` 的套件相依明確
> `Conflicts: gazebo (>= 11.0.0)`。裝 Garden 會移除 Classic。

---

## 內容

| 檔案 | 說明 |
|---|---|
| `src/offboard_takeoff_node.cpp` | 主體：起飛 → 懸停 → 降落的狀態機 |
| `include/drone_control/offboard_takeoff_node.hpp` | 類別定義，供之後的編隊節點複用 |
| `launch/single_drone.launch.py` | 單機 |
| `launch/three_drones.launch.py` | 三機（MAV1 / MAV2 / MAV3） |
| `scripts/start_3_px4.sh` | 一次啟動三台 PX4 SITL |
| `scripts/check_env.sh` | 環境健檢，唯讀 |
| `px4_deps.repos` | 記錄 px4_msgs 版本，供 `vcs import` 還原 |

---

## 安裝

### 情境 A：已經有 ROS 2 workspace（飛場 / 學校電腦）

環境已由別人裝好時，三行就能跑：

```bash
cd ~/ros2_ws/src
git clone https://github.com/Oliver-17/fanros2_ws.git drone_control
cd ~/ros2_ws && colcon build --packages-select drone_control
```

> `--packages-select drone_control` 只編這一個 package。
> 共用的 workspace 直接下 `colcon build` 會把別人的 package 全部重編，
> 很浪費，而且可能編壞別人的東西。

編完先健檢一次，確認版本對得上：

```bash
./src/drone_control/scripts/check_env.sh
```

**最需要確認的是 `px4_msgs` 的分支**（期望 `release/1.14` @ `ffb6e80`）。
版本不符時**編譯會過、執行才錯亂**，極難除錯。

`px4_msgs` 不存在的話才直接 import：

```bash
cd ~/ros2_ws && vcs import src < src/drone_control/px4_deps.repos
```

> ⚠️ **已經存在但版本不同時，先不要 import。** 共用的 workspace 裡
> `src/px4_msgs` 是大家共用的一份，切版本會影響同學。
> 處理方式見下方〈共用電腦注意事項〉。

### 情境 B：全新的電腦

> **`git clone` 只會帶來原始碼，不會帶來環境。**
> ROS 2、PX4、Gazebo、XRCE Agent 都是裝在系統上的東西，不在這個 repo 裡。
> 一台乾淨的電腦要走完全部步驟，**預計 1~2 小時**（PX4 編譯就佔 20~40 分鐘）。

#### 步驟 0：建 workspace、取得本 package、健檢

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Oliver-17/fanros2_ws.git drone_control
./drone_control/scripts/check_env.sh
```

腳本只讀不寫，會列出缺少的項目和對應指令。**已經齊全的步驟就跳過。**

#### 步驟 1：ROS 2 Humble

見 <https://docs.ros.org/en/humble/Installation.html>（`ros-humble-desktop`）。

#### 步驟 2：Gazebo Garden

```bash
sudo apt update && sudo apt install -y lsb-release gnupg wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
     -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install -y gz-garden
```

> ⚠️ 這會**移除** Gazebo Classic 與 `ros-humble-gazebo-*`（`gz-tools2` 明確宣告
> `Conflicts: gazebo (>= 11.0.0)`）。如果這台電腦有其他人的專案在用 Classic，
> **裝之前務必先問過。**

#### 步驟 3：PX4-Autopilot

```bash
git clone -b v1.14.0 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh          # 裝編譯相依

# 這幾個 Python 套件官方腳本不一定會裝齊，缺了會在編譯途中才爆
pip3 install --user kconfiglib jinja2 empy pyros-genmsg jsonschema

make px4_sitl_default                  # 20~40 分鐘
```

編完務必確認 `gz_bridge` 有被包進去，否則 Gazebo 裡永遠不會出現飛機：

```bash
strings build/px4_sitl_default/bin/px4 | grep -x gz_bridge
```

沒有輸出代表編譯時找不到 gz-transport（`CMakeCache.txt` 裡會是
`gz-transport_DIR-NOTFOUND`）。此時要 `make clean` 後重編，而且**要先裝好 Gazebo**。

#### 步驟 4：Micro XRCE-DDS Agent

```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/src_tools/Micro-XRCE-DDS-Agent
cd ~/src_tools/Micro-XRCE-DDS-Agent && mkdir -p build && cd build
# 必須加這個 flag：官方預設會去 checkout 已被 eProsima 刪除的 Fast-DDS 2.12.x 分支
cmake .. -DUAGENT_USE_SYSTEM_FASTDDS=ON
make -j$(nproc) && sudo make install && sudo ldconfig
```

> 因為改用系統的 Fast-DDS（ROS Humble 的 2.6），
> **執行 Agent 前必須先 `source /opt/ros/humble/setup.bash`**，否則找不到動態函式庫。

#### 步驟 5：取得 px4_msgs 並編譯

```bash
sudo apt install -y python3-vcstool
cd ~/ros2_ws
vcs import src < src/drone_control/px4_deps.repos    # release/1.14 @ ffb6e80
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

#### 步驟 6：再健檢一次

```bash
./src/drone_control/scripts/check_env.sh   # 應該全部 ✓
```

> **沒有 sudo 權限怎麼辦？** 步驟 1、2 一定要 root。如果學校電腦不給 sudo，
> 就得請管理員先裝好 ROS 2 Humble 和 Gazebo Garden；
> 步驟 3、4、5 都裝在家目錄，一般帳號就能做（步驟 4 的 `sudo make install`
> 可改成 `cmake -DCMAKE_INSTALL_PREFIX=$HOME/.local ..`）。

---

## 執行：單機

需要 **3 個終端**。

```bash
# 終端 1 — Micro XRCE-DDS Agent（PX4 與 ROS 2 之間的翻譯官，UDP 8888）
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

```bash
# 終端 2 — PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

```bash
# 終端 3 — 控制節點
source ~/ros2_ws/install/setup.bash
ros2 launch drone_control single_drone.launch.py
```

### 單機可選參數

```bash
ros2 launch drone_control single_drone.launch.py \
    vehicle_name:=MAV1 \
    use_namespace:=true \
    takeoff_altitude:=2.0 \
    hover_duration:=10.0
```

---

## 執行：三機（MAV1 / MAV2 / MAV3）

需要 **3 個終端**。

```bash
# 終端 1 — Agent（同上，一個 Agent 服務三台）
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

```bash
# 終端 2 — 一次啟動三台 PX4（會等到三台都就緒才印出 banner）
~/ros2_ws/src/drone_control/scripts/start_3_px4.sh
```

```bash
# 終端 3 — 等終端 2 印出「三台全部就緒」再執行
source ~/ros2_ws/install/setup.bash
ros2 launch drone_control three_drones.launch.py
```

三台的 namespace / MAV_SYS_ID / 起飛高度定義在 `three_drones.launch.py` 的 `FLEET` 常數。

---

## 共用電腦注意事項

環境已經裝好時，clone 進 `src/` 再 `colcon build --packages-select drone_control`
大約 5 分鐘就能跑。以下是多人共用同一台機器時，跟環境無關但更容易出事的幾件事。

> 本專案的實際使用情境是**一個飛場、三台飛機、一次一個人輪流用**。
> 「輪流」不會有同時搶資源的問題，但下面第一項反而更容易中 ——
> 交接時沒有人會去清背景行程。

### 接手時的交接檢查（輪流使用必做）

上一位同學把終端關掉，**不代表行程結束**。`gz sim` server 和 `px4`
都是背景行程，Ctrl+C 關終端後仍會繼續跑。此時你啟動 PX4，
它偵測到「世界已存在」就不會開 GUI，你的飛機會生在一個看不見的世界裡
（見下方「Gazebo 視窗沒出現」）。

```bash
# 1. 先看有沒有殘留
ps -eo user,pid,etime,args | grep -E "px4|gz sim|MicroXRCEAgent" | grep -v grep

# 2. 確認 user 與 etime 之後再清（別殺到還在用的人）
pkill -x px4
pkill -f "gz sim"          # 必須 -f，gz 是 Ruby 包裝腳本，-x 抓不到
pkill -x MicroXRCEAgent
```

### ROS_DOMAIN_ID 建議固定設一個自己的號碼

ROS 2 預設 `ROS_DOMAIN_ID=0`，而 DDS 的發現機制是**走網路 multicast，不是只看本機**。

所以「飛場一次一個人」不等於「網路上只有一個 ROS 2」：
同學在座位上用自己的電腦跑 SITL，只要在同一個網段，
他的 topic 就會跟你的互相發現、混在一起 —— 而且**不會有任何錯誤訊息**，
DDS 認為這完全合法。真機階段撞到的代價是炸機，不是重開模擬。

```bash
export ROS_DOMAIN_ID=42      # 0~101，跟同學喬好不要重複；每個終端都要設
ros2 node list               # 驗證：只該看到自己的節點
```

### UDP 8888 被佔用時（同時使用才會遇到）

別人的 Agent 先綁走 8888 的話，你的會啟動失敗。兩邊都要改成同一個新 port：

```bash
MicroXRCEAgent udp4 -p 9888                          # 終端 1
PX4_UXRCE_DDS_PORT=9888 make px4_sitl gz_x500        # 終端 2
```

> port 的來源是 `ROMFS/px4fmu_common/init.d-posix/rcS:285`，
> 預設 8888，可由環境變數 `PX4_UXRCE_DDS_PORT` 覆寫。

### px4_msgs 版本要先查，不要直接覆蓋

「同學跑過」不代表版本跟你一樣。px4_msgs 若是 `main` 或 `release/1.15`，
欄位定義跟 PX4 v1.14 對不上時**編譯會過、執行才錯亂**（數值變垃圾），極難除錯。

**先查，別急著 import：**

```bash
git -C ~/ros2_ws/src/px4_msgs describe --tags    # 期望 v1.14.0
git -C ~/ros2_ws/src/px4_msgs rev-parse --short HEAD   # 期望 ffb6e80
```

| 結果 | 怎麼辦 |
|---|---|
| 就是 `v1.14.0` / `ffb6e80` | 什麼都不用做 |
| **不存在** | 可以安全 import：`cd ~/ros2_ws && vcs import src < src/drone_control/px4_deps.repos` |
| **版本不同** | ⚠️ **先不要動**，見下方 |

> ⚠️ 共用的 workspace 裡，`src/px4_msgs` 是**大家共用的一份**。
> 直接 `vcs import` 會把它切到別的版本，同學的 package 可能就編不過或行為改變。
> 遇到版本不同時，先跟使用那台電腦的人確認能不能統一到 `release/1.14`；
> 不方便更動的話，就在自己家目錄另開一個 workspace，不要動公用的那個：
>
> ```bash
> mkdir -p ~/my_ws/src && cd ~/my_ws/src
> git clone https://github.com/Oliver-17/fanros2_ws.git drone_control
> cd ~/my_ws && vcs import src < src/drone_control/px4_deps.repos
> colcon build
> ```

---

## 常見問題

### Gazebo 視窗沒出現，但終端訊息一切正常

有殘留的 Gazebo server 還在背景跑。PX4 的 `px4-rc.simulator` 偵測到世界已存在，
就不會再開 GUI，你的飛機會靜靜生在那個看不見的世界裡。

```bash
pkill -x px4
pkill -f "gz sim"     # 必須用 -f
```

> **`pkill -x gz` 抓不到。** `gz` 是一個 Ruby 包裝腳本，
> 行程的 `comm` 是 `ruby` 而不是 `gz`，精確比對永遠不會命中。

### 訂閱 `/fmu/out/*` 收不到任何資料

QoS 不相容。PX4 以 **BEST_EFFORT** 發布，rclcpp 預設訂閱是 **RELIABLE**，
兩者不相容時 DDS 不會報錯，只是**安靜地什麼都收不到**。

```cpp
rclcpp::QoS qos(rclcpp::KeepLast(5));
qos.best_effort();
qos.durability_volatile();
```

> `ros2 topic echo` 會自動配合對方的 QoS，所以用 echo 測「看起來正常」，
> 這會誤導你以為問題不在 QoS。

### 高度數值跟實際差了幾十公分

`vehicle_local_position.z` 的基準是 **EKF2 原點**，不是地面。
`gz_x500` 機型 `SENS_EN_BAROSIM 0` / `SENS_EN_GPSSIM 1`，高度來自模擬 GPS，
原點會落在一個浮動的偏移上（實測 0.08 ~ 1.04 m）。

> 已知議題，尚未修正。修法是起飛前把 `z` 一起鎖進 `takeoff_down_`，改用相對高度。

---

## 座標系備忘

PX4 用 **NED**：X=北, Y=東, **Z=下（正值向下）**。
所以「起飛到 2 公尺」是 `position[2] = -2.0`。

ROS 慣例是 **ENU** + 機體 **FLU**；PX4 是 **NED** + 機體 **FRD**。
`VehicleOdometry.q` 的定義是「FRD 機體系 → 參考系」的旋轉，
所以做橋接時**位置和四元數都要轉**，只轉位置是常見 bug。

---

## 待辦

- [ ] 高度基準改為相對起飛點
- [ ] `/fleet/status` 編隊同步（三機一起起降）
- [ ] `mocap_px4_bridge`：OptiTrack VRPN → `/fmu/in/vehicle_visual_odometry`
- [ ] 真機（Pix32 v6 + Pi4，Agent 走 serial）
