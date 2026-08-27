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
| `scripts/check_env.sh` | 環境健檢，唯讀。真機用 `--flight` |
| `scripts/record_flight.sh` | 用 `ros2 bag` 錄飛行資料，事後比對指令與實際 |
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

## 執行：總覽

| 情境 | 終端數 | PX4 跑在哪 | Agent 連線方式 | 狀態 |
|---|---|---|---|---|
| 模擬 · 單機 | 3 | 你的電腦 | UDP 8888 | ✅ 已驗證 |
| 模擬 · 三機 | 3 | 你的電腦 ×3 | UDP 8888（共用） | ✅ 已驗證 |
| 實機 · 單機 | 2 | Pix32 v6 | 序列埠 | ⏳ 待測 |
| 實機 · 三機 | 2 | Pix32 v6 ×3 | 序列埠 ×3 | ⛔ 尚未開始 |

> **不論哪個情境，控制節點的程式碼完全相同。** 差別只在「PX4 從哪裡來」。

---

## 執行：模擬 (SITL) — 單機

需要 **3 個終端**，順序不能顛倒。

```bash
# 終端 1 — Micro XRCE-DDS Agent（PX4 與 ROS 2 之間的翻譯官）
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

```bash
# 終端 2 — PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**檢查點**：看到 `pxh>` 提示字元後，回終端 1 應該出現 `create_topic … vehicle_local_position`。
沒出現就是沒連上，不要往下走。

```bash
# 終端 3 — 控制節點
source ~/ros2_ws/install/setup.bash
ros2 launch drone_control single_drone.launch.py
```

### 可選參數

| 參數 | 預設 | 說明 |
|---|---|---|
| `takeoff_altitude` | `0.8` | 起飛高度（公尺），**相對起飛點** |
| `position_tolerance` | `0.3` | 高度到達的容忍值，**不要超過起飛高度的 1/3** |
| `hover_duration` | `10.0` | 到達後懸停秒數 |
| `vehicle_name` | `MAV1` | 只影響 log 顯示 |
| `use_namespace` | `false` | PX4 有帶 `PX4_UXRCE_DDS_NS` 時要設 `true` |
| `target_system` | `1` | MAVLink system id = PX4 instance + 1 |

低空測試（飛場空間有限時）：

```bash
ros2 launch drone_control single_drone.launch.py \
    takeoff_altitude:=0.5 \
    position_tolerance:=0.15
```

拉高測試：

```bash
ros2 launch drone_control single_drone.launch.py \
    takeoff_altitude:=2.0 \
    hover_duration:=15.0
```

### 停止

```bash
pkill -x px4 ; pkill -f "gz sim"
```

> 第二行**必須用 `-f`**：`gz` 是 Ruby 包裝腳本，行程名是 `ruby`，`-x` 永遠抓不到。

---

## 執行：模擬 (SITL) — 三機

需要 **3 個終端**。終端 1 的 Agent 跟單機完全一樣，**一個 Agent 服務三台**。

```bash
# 終端 1 — Agent
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

```bash
# 終端 2 — 一次啟動三台 PX4（腳本會逐台確認就緒才起下一台）
~/ros2_ws/src/drone_control/scripts/start_3_px4.sh

# 電腦跑不動時關掉畫面：
HEADLESS=1 ~/ros2_ws/src/drone_control/scripts/start_3_px4.sh
```

**等它印出這段才可以往下走：**

```
==================================================
 三台全部就緒 — 現在可以去終端 3 跑：
==================================================
```

> ⚠️ **提早跑終端 3 會出事**：先連上的那台會自己起飛，
> 另外兩台還卡在等 PX4，三台動作完全錯開。

```bash
# 終端 3 — 控制節點
source ~/ros2_ws/install/setup.bash
ros2 launch drone_control three_drones.launch.py
```

### 三台的對應關係

| 角色 | instance | `PX4_UXRCE_DDS_NS` | topic 前綴 | `MAV_SYS_ID` | 起飛位置 (N,E) | 高度 |
|---|---|---|---|---|---|---|
| MAV1 長機 | `-i 0` | `MAV1` | `/MAV1/fmu/…` | 1 | (0, 0) | 2.0 m |
| MAV2 僚機 | `-i 1` | `MAV2` | `/MAV2/fmu/…` | 2 | (0, 3) | 2.5 m |
| MAV3 僚機 | `-i 2` | `MAV3` | `/MAV3/fmu/…` | 3 | (0, −3) | 3.0 m |

高度刻意錯開，避免水平漂移時互撞。定義在 `three_drones.launch.py` 的 `FLEET` 常數，
**飛場高度不夠時要先改小**。容忍值可從命令列覆寫：

```bash
ros2 launch drone_control three_drones.launch.py position_tolerance:=0.2
```

> 目前三台是**各自獨立**跑完起飛→懸停→降落，彼此不溝通。
> 真正的編隊同步還沒實作（見〈待辦〉）。

---

## 執行：實機 — 單機

> ⚠️ **上真機前務必先讀〈真機注意事項〉。** 有一項是程式碼還沒改、會跟飛手搶控制權的。

### 一次性設定（每台飛機只需做一次）

**① QGroundControl 參數**（`Vehicle Setup → Parameters`，改完 `Tools → Reboot Vehicle`）

| 參數 | 值 | 說明 |
|---|---|---|
| `UXRCE_DDS_CFG` | `Disabled` | 改用 `extras.txt` 啟動，避免兩個 client 搶序列埠 |
| `UXRCE_DDS_DOM_ID` | `42` | **必須等於 Pi4 上的 `ROS_DOMAIN_ID`** |
| `UXRCE_DDS_KEY` | `1` | 多機時每台必須不同且不為 0 |
| `SER_TEL1_BAUD` | `921600` | 對應你選的 TELEM port |
| `MAV_SYS_ID` | `1` | 多機時每台必須不同 |

**② 飛控 SD 卡** — 建立 `/fs/microsd/etc/extras.txt`：

```sh
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyS5 -b 921600 -n MAV1
```

> `/dev/ttyS5` 是 **Pix32 v6 (FMUv6C) 的 TELEM 1**，來源
> `boards/px4/fmu-v6c/default.px4board`：`CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS5"`
> （TELEM 2 是 `/dev/ttyS3`）。
>
> **namespace 只能用 `-n` 命令列旗標指定，QGC 裡找不到對應參數。**
> SITL 用的 `PX4_UXRCE_DDS_NS` 只存在於 `init.d-posix/rcS`，真機那條路徑沒有。

### 每次飛行

需要 **2 個終端，兩個都在 Pi4 上**（不是你的筆電）。PX4 已經在飛控裡跑著，不用啟動。

```bash
# 終端 1 — Agent（走序列埠）
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

> **兩個裝置名不要搞混**：`-d /dev/ttyS5` 是**飛控那側**（寫在 extras.txt 裡），
> `--dev /dev/ttyUSB0` 是 **Pi4 這側**。
> Pi4 的裝置名視接法而定：USB 轉接線通常是 `/dev/ttyUSB0`，
> 直接接 GPIO UART 是 `/dev/serial0`。插拔前後各跑一次 `ls /dev/tty*` 比對最準。

```bash
# 終端 2 — 控制節點
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch drone_control single_drone.launch.py \
    takeoff_altitude:=0.8 \
    position_tolerance:=0.2
```

> **節點必須跑在 Pi4 上，不能跑在筆電上。**
> `COM_OF_LOSS_T` 預設 **1.0 秒** —— 超過 1 秒沒收到 setpoint 就觸發失效保護。
> wifi 抖一下就會掉出 Offboard。

### 起飛前檢查

```bash
export ROS_DOMAIN_ID=42                       # 每個終端都要
./scripts/check_env.sh --flight               # 真機模式，跳過 Gazebo/PX4 原始碼檢查
ros2 topic list | grep fmu                    # 應該看得到 /fmu/in/… 與 /fmu/out/…
ros2 topic echo /fmu/out/vehicle_status --once | grep -E "nav_state|arming_state"
```

| 檢查 | 期望 |
|---|---|
| `ros2 topic list` 看得到 `/fmu/*` | Agent 與飛控連上了 |
| `arming_state` | `1`（DISARMED） |
| 遙控器 | 已綁定，Offboard 開關在**關閉**位置 |
| 電池 | 電壓正常，QGC 無警告 |
| 螺旋槳 | **先不要裝**，第一次只驗證 topic 通不通 |

### 錄下這次飛行

```bash
./scripts/record_flight.sh          # 存到 ~/flight_logs/<時間戳>/
```

---

## 執行：實機 — 三機（尚未測試）

> ⛔ **還沒開始。** 先把實機單機跑穩再進到這裡。
> 以下只是把已知的設定差異記下來，**未經驗證**。

### 三台的參數（其餘與單機相同）

| 參數 | MAV1 | MAV2 | MAV3 |
|---|---|---|---|
| `UXRCE_DDS_KEY` | 1 | 2 | 3 |
| `MAV_SYS_ID` | 1 | 2 | 3 |
| `UXRCE_DDS_DOM_ID` | 42 | 42 | 42 |
| `extras.txt` 的 `-n` | `MAV1` | `MAV2` | `MAV3` |

> `UXRCE_DDS_KEY` 的官方說明：*"must be different from zero. In a single agent -
> multi client configuration, each client must have a unique session key."*
> **三台撞 key，Agent 會當成同一台，topic 直接亂掉。**

### 未解決的問題

- **一個 Agent 還是三個？** 模擬時三台 PX4 共用一個 UDP Agent，
  但真機是三條獨立的序列埠。可能要在 Pi4 上跑三個 Agent（各自 `--dev`），
  或每台飛機配一台 Pi4。**尚未確認。**
- **三台的起飛高度**要依飛場淨空高度重新設定，`FLEET` 裡的 2.0/2.5/3.0 是模擬用的。
- **編隊同步**還沒實作，目前三台各飛各的。

---

## 真機注意事項

### ⚠️ 程式碼待修：會跟飛手搶控制權

`handleRequestOffboard()`（`src/offboard_takeoff_node.cpp:315`）在切模式失敗時**會自動重試**。

模擬沒有遙控器，這個設計沒問題。但真機上飛手撥開關想拿回控制時，
**節點會把模式又切回 Offboard** —— 這是會出事的。

> **上真機前必須修掉。** 尚未處理，見〈待辦〉。

### 遙控器要先設好 Offboard 開關

`COM_FLTMODE1` ~ `COM_FLTMODE6` 其中一個設成 **`7`**（Offboard）。
飛手撥過去才交給程式，撥回來立刻收回控制權。

| 值 | 模式 | | 值 | 模式 |
|---|---|---|---|---|
| 0 | Manual | | **7** | **Offboard** |
| 1 | Altitude | | **8** | **Stabilized** |
| 2 | Position | | 10 | Takeoff |
| 4 | Hold | | 11 | Land |

### 建議的推進順序

1. 不裝螺旋槳，只驗證 `ros2 topic list` 看得到 `/fmu/*`
2. 不裝螺旋槳，跑一次完整流程，看 log 狀態機有沒有走完
3. 裝螺旋槳、綁繩、`takeoff_altitude:=0.5`
4. 逐步拉高
5. 三機

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

### 已 ARM、螺旋槳有轉，但飛機不動

**高度基準的問題，已於 2026-08-27 修正。** 若你的版本較舊會遇到：

`vehicle_local_position.z` 的基準是 **EKF2 原點，不是地面**。實測 SITL 剛開機時
飛機停在地上，`z` 卻是 `-0.42`。舊版直接用 `-takeoff_altitude_` 當絕對目標，
於是：

```
目標 0.5 m，飛機「已在」0.42 m，誤差 0.08 < 容忍值 0.3  →  判定「已到達」
```

飛機只需爬 8 公分，推力不足以離地，看起來就是「螺旋槳空轉」。

**修法**（已套用）：起飛前把 `z` 一起鎖進 `takeoff_down_`，之後全部改用相對高度。

```cpp
takeoff_down_    = local_position_.z;                        // 鎖定「地面」
target_down      = takeoff_down_ - takeoff_altitude_;        // 目標
current_altitude = takeoff_down_ - local_position_.z;        // 目前高度
```

> 若仍卡在爬升，多半是 `position_tolerance` 設得太小
> （氣壓計雜訊約 ±0.15 m）。`handleTakeoff()` **沒有逾時機制**，
> 會一直懸停在目標高度不降落 —— `Ctrl+C` 讓失效保護接手即可。

---

## 座標系備忘

PX4 用 **NED**：X=北, Y=東, **Z=下（正值向下）**。
所以「起飛到 2 公尺」是 `position[2] = -2.0`。

ROS 慣例是 **ENU** + 機體 **FLU**；PX4 是 **NED** + 機體 **FRD**。
`VehicleOdometry.q` 的定義是「FRD 機體系 → 參考系」的旋轉，
所以做橋接時**位置和四元數都要轉**，只轉位置是常見 bug。

---

## 待辦

- [x] 高度基準改為相對起飛點（2026-08-27）
- [ ] `handleRequestOffboard()` 自動重試會跟飛手搶控制權，上真機前必修
- [ ] `handleTakeoff()` 加逾時保護
- [ ] `/fleet/status` 編隊同步（三機一起起降）
- [ ] `mocap_px4_bridge`：OptiTrack VRPN → `/fmu/in/vehicle_visual_odometry`
- [ ] 真機（Pix32 v6 + Pi4，Agent 走 serial）
