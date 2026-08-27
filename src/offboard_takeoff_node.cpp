// =============================================================================
//  offboard_takeoff_node.cpp
//
//  對應 PX4-Autopilot v1.14.0（uXRCE-DDS）+ px4_msgs release/1.14
//
//  ---------------------------------------------------------------------------
//  【必讀觀念一】NED 座標系 —— 高度是負的！
//  ---------------------------------------------------------------------------
//  PX4 的本地座標系是 NED：
//        N (North) = X 軸，指向「北」
//        E (East)  = Y 軸，指向「東」
//        D (Down)  = Z 軸，指向「地心」  <<<< 重點在這
//
//  所以 Z 軸是「往下為正」。要飛到離地 2 公尺，setpoint 的 z 要填 -2.0。
//  這是初學者第一大坑：如果你填 +2.0，飛機會往「地下」飛，
//  在模擬器裡看起來就是「解鎖後貼地掙扎、翻覆」，在真機上就是直接撞地。
//
//      想飛高 2 公尺  ->  position[2] = -2.0f
//      目前高度 2 公尺 ->  local_position.z 讀到的是 -2.0 左右
//      離地高度       ->  -local_position.z
//
//  另外要注意：這個座標系的「原點」是 EKF2 濾波器啟動當下飛機所在的位置，
//  不是 GPS home、也不是世界原點。所以 x=0, y=0 就代表「原地」。
//
//  ---------------------------------------------------------------------------
//  【必讀觀念二】QoS 為什麼一定要設 BEST_EFFORT
//  ---------------------------------------------------------------------------
//  ROS 2 底層是 DDS，每個 topic 都有 QoS（服務品質）設定，其中 reliability 有兩種：
//        RELIABLE    = 保證送達，掉包會重傳
//        BEST_EFFORT = 盡力而為，掉包就算了
//
//  PX4 的 uXRCE-DDS client 在發布 /fmu/out/* 時，用的是 BEST_EFFORT
//  （因為飛控資料是高頻串流，舊資料重傳沒有意義，還會塞爆頻寬）。
//
//  而 ROS 2 的 rclcpp 預設 QoS 是 RELIABLE。
//
//  DDS 的相容規則是「訂閱端的要求不能比發布端提供的更嚴格」：
//        發布 RELIABLE    + 訂閱 BEST_EFFORT  -> OK
//        發布 BEST_EFFORT + 訂閱 RELIABLE     -> 不相容，連線建不起來
//
//  所以如果你用預設 QoS 去訂閱 /fmu/out/vehicle_local_position，
//  結果是「完全收不到任何訊息」——而且 ROS 2 不會噴錯，
//  `ros2 topic echo` 也看得到資料（因為 echo 有自動 QoS 匹配），
//  只有你自己的 node 安安靜靜地收不到東西。這是超級難 debug 的經典陷阱。
//
//  結論：訂閱 /fmu/out/* 一律用 BEST_EFFORT。
//
//  反過來，我們「發布」到 /fmu/in/* 時用預設的 RELIABLE 就好，
//  因為 RELIABLE 的發布端可以相容 PX4 那邊任何設定的訂閱端。
// =============================================================================

#include "drone_control/offboard_takeoff_node.hpp"

#include <cmath>

namespace drone_control
{

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// 控制迴圈頻率。PX4 要求 Offboard 心跳不能低於 2Hz，否則 0.5 秒內就會觸發 failsafe。
// 官方建議、也是大家慣用的值是 10Hz，留有足夠餘裕。
static constexpr int    kLoopRateHz   = 10;
static constexpr auto   kLoopPeriod   = 100ms;

// 切到 Offboard 之前，要先「預熱」發送多久的 setpoint。
// PX4 的規則：收不到穩定的 setpoint 串流就拒絕進入 Offboard 模式。
// 1 秒 = 10 個 loop，這也是你需求裡「發送滿 1 秒後才切模式」的由來。
static constexpr int    kWarmupLoops  = kLoopRateHz * 1;

// 「到達目標高度」的條件必須連續成立幾個週期才算數。
//
// 為什麼不是一達標就切狀態？
//   爬升接近目標時，飛機會先減速再穩定。減速的那一瞬間垂直速度會暫時很小，
//   如果剛好高度誤差也擦邊通過容忍值，就會「提早宣告到達」——
//   實測曾在 1.70 m 就判定到達 2.00 m（雖然之後懸停時自己修正回來了）。
//   單機看不出問題，但三機編隊時每台提早的時機不同，隊形會歪。
//   要求連續成立 5 個週期（0.5 秒），就能確保是真的穩住而不是路過。
static constexpr int    kSettledLoops = 5;

// 指令（切模式 / Arm）重送的間隔與上限。
static constexpr int    kRetryLoops   = kLoopRateHz / 2;   // 每 0.5 秒重送一次
static constexpr int    kMaxRetries   = 20;                // 最多試 10 秒

// -----------------------------------------------------------------------------
// 建構子
// -----------------------------------------------------------------------------
OffboardTakeoffNode::OffboardTakeoffNode()
: Node("offboard_takeoff")
{
  // ---------------------------------------------------------------------------
  // 1) 宣告參數
  //
  //    為什麼要用參數而不是寫死字串？
  //    因為 PX4 SITL 跑多機時，第 0 台的 topic 是 /fmu/in/...，
  //    第 1、2 台會自動加上 namespace 變成 /px4_1/fmu/in/...、/px4_2/fmu/in/...
  //    （出處：PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS 第 267-272 行，
  //      當 px4_instance != 0 時會給 uxrce_dds_client 加上 "-n px4_$px4_instance"）
  //
  //    同時每台的 MAVLink system id 也不一樣（rcS 第 132 行：UXRCE_DDS_KEY = instance+1），
  //    VehicleCommand 的 target_system 必須填對，否則指令會被別台飛機忽略或誤收。
  //
  //    把這兩個做成參數，之後三機編隊就是同一支執行檔開三份、餵不同參數，
  //    這個 .cpp 一行都不用改。
  // ---------------------------------------------------------------------------
  // vehicle_name 只是「這台飛機叫什麼」，用於 log 辨識，不影響任何 topic 名稱。
  // px4_namespace 才是真正決定 topic 前綴的東西，它必須與 PX4 啟動時的
  // PX4_UXRCE_DDS_NS 環境變數一致，否則會訂閱到不存在的 topic。
  vehicle_name_       = this->declare_parameter<std::string>("vehicle_name", "MAV1");
  px4_namespace_      = this->declare_parameter<std::string>("px4_namespace", "");
  target_system_      = this->declare_parameter<int>("target_system", 1);
  takeoff_altitude_   = this->declare_parameter<double>("takeoff_altitude", 2.0);
  hover_duration_     = this->declare_parameter<double>("hover_duration", 10.0);
  position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.3);
  // PX4 v1.16+ 的 /fmu/out/ topic 帶版本後綴，v1.14 沒有。空字串 = 舊版行為。
  topic_suffix_       = this->declare_parameter<std::string>("topic_suffix", "");

  // ---------------------------------------------------------------------------
  // 2) 設定 QoS
  // ---------------------------------------------------------------------------

  // 訂閱 PX4 -> ROS 的資料，必須 BEST_EFFORT（理由見檔案開頭的長篇說明）。
  //   KeepLast(5)         : 只留最新 5 筆，飛控資料舊了就沒用
  //   best_effort()       : 關鍵！配合 PX4 的發布設定
  //   durability_volatile(): 不保留歷史資料給晚加入的訂閱者
  rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(5));
  px4_sub_qos.best_effort();
  px4_sub_qos.durability_volatile();

  // 發布 ROS -> PX4 的指令，用預設的 RELIABLE 即可（相容性最好）。
  rclcpp::QoS px4_pub_qos(rclcpp::KeepLast(10));

  // ---------------------------------------------------------------------------
  // 3) 建立 publisher / subscriber
  //    topic 名稱一律用 namespace 前綴組出來
  // ---------------------------------------------------------------------------
  const std::string ns = px4_namespace_;   // "" 或 "/px4_1"

  offboard_mode_pub_ = this->create_publisher<OffboardControlMode>(
    ns + "/fmu/in/offboard_control_mode", px4_pub_qos);

  setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
    ns + "/fmu/in/trajectory_setpoint", px4_pub_qos);

  command_pub_ = this->create_publisher<VehicleCommand>(
    ns + "/fmu/in/vehicle_command", px4_pub_qos);

  local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
    ns + "/fmu/out/vehicle_local_position" + topic_suffix_, px4_sub_qos,
    std::bind(&OffboardTakeoffNode::onLocalPosition, this, std::placeholders::_1));

  vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
    ns + "/fmu/out/vehicle_status" + topic_suffix_, px4_sub_qos,
    std::bind(&OffboardTakeoffNode::onVehicleStatus, this, std::placeholders::_1));

  // ---------------------------------------------------------------------------
  // 4) 啟動 10Hz 控制迴圈
  // ---------------------------------------------------------------------------
  timer_ = this->create_wall_timer(
    kLoopPeriod, std::bind(&OffboardTakeoffNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "==============================================");
  RCLCPP_INFO(this->get_logger(), " Offboard 起飛節點已啟動 — %s", vehicle_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "   topic 前綴     : '%s'%s",
              px4_namespace_.c_str(), px4_namespace_.empty() ? " (單機模式)" : "");
  RCLCPP_INFO(this->get_logger(), "   target_system  : %d", target_system_);
  // 把完整 topic 名稱印出來：卡在 WAIT_FOR_FCU 時，
  // 直接拿這兩行去跟 `ros2 topic list` 比對就知道是不是名字錯了。
  RCLCPP_INFO(this->get_logger(), "   訂閱位置       : %s%s%s",
              px4_namespace_.c_str(), "/fmu/out/vehicle_local_position",
              topic_suffix_.c_str());
  RCLCPP_INFO(this->get_logger(), "   訂閱狀態       : %s%s%s",
              px4_namespace_.c_str(), "/fmu/out/vehicle_status",
              topic_suffix_.c_str());
  RCLCPP_INFO(this->get_logger(), "   起飛高度       : %.2f m（相對起飛點，不是絕對高度）",
              takeoff_altitude_);
  RCLCPP_INFO(this->get_logger(), "   懸停時間       : %.1f s", hover_duration_);
  RCLCPP_INFO(this->get_logger(), "==============================================");
  RCLCPP_INFO(this->get_logger(), "[狀態] WAIT_FOR_FCU — 等待 PX4 位置資料…");
  RCLCPP_INFO(this->get_logger(),
              "  (若一直卡在這裡，代表 XRCE Agent 沒起來、或 PX4 SITL 沒連上)");
}

// -----------------------------------------------------------------------------
// 訂閱回呼：只負責存資料，判斷邏輯全部集中在狀態機裡，避免多執行緒的競態問題。
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::onLocalPosition(const VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
  local_position_received_ = true;
}

void OffboardTakeoffNode::onVehicleStatus(const VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
  vehicle_status_received_ = true;
}

// -----------------------------------------------------------------------------
// 主控制迴圈（10Hz）
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::controlLoop()
{
  ++loop_count_;

  // ---------------------------------------------------------------------------
  // 飛手接管偵測 —— 真機安全機制，優先於所有其他邏輯
  //
  //   條件：曾經成功進入 Offboard，但現在 nav_state 已經不是 Offboard 了。
  //   代表飛手撥了遙控器開關，主動把控制權拿回去。
  //
  //   為什麼一定要處理？如果放著不管，狀態機會繼續往下跑，
  //   懸停計時到了就送 VEHICLE_CMD_NAV_LAND —— 飛手正在手動飛，
  //   我們卻叫飛機降落。這是搶控制權，真機上會出事。
  //
  //   收手的方式是「什麼都不做」：不發心跳、不送指令、直接進 DONE。
  //   PX4 會維持飛手選的模式，控制權完整交還。
  //
  //   LANDING 要排除：那是我們自己交棒給 AUTO_LAND，
  //   nav_state 本來就會變成 18，不是飛手接管。
  // ---------------------------------------------------------------------------
  const bool pilot_took_over =
    offboard_confirmed_ &&
    (state_ != FlightState::LANDING) &&
    (state_ != FlightState::DONE) &&
    (vehicle_status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD);

  if (pilot_took_over) {
    RCLCPP_WARN(this->get_logger(),
                "偵測到飛手接管（nav_state=%d，已離開 Offboard）。"
                "節點立刻停止所有輸出，控制權完全交還飛手。",
                vehicle_status_.nav_state);
    transitionTo(FlightState::DONE, "飛手接管，節點中止");
    return;   // 這個 tick 起不再發送任何東西
  }

  // ---------------------------------------------------------------------------
  // 心跳：只要還在 Offboard 流程中，每個 tick 都必須發送
  //   OffboardControlMode + TrajectorySetpoint 這一「對」訊息。
  //
  //   為什麼一定要成對？
  //     OffboardControlMode 告訴 PX4「我要用位置控制這一層」
  //     TrajectorySetpoint  提供該層實際的目標值
  //   只發前者沒有目標值、只發後者 PX4 不知道要用哪層 —— 都會被拒絕。
  //
  //   為什麼要「持續」發？
  //     PX4 若超過 COM_OF_LOSS_T（預設 1.0 秒）收不到心跳，會判定地面站失聯，
  //     自動跳出 Offboard 進入 failsafe（通常是 Hold 或 Land）。
  //     所以就算飛機已經到位在懸停，心跳也不能停。
  //
  //   例外：進入 LANDING 之後，飛控已經被我們交棒給 PX4 的自動降落模式
  //   （nav_state = AUTO_LAND），這時就不該再發 Offboard 心跳去跟它搶控制權。
  // ---------------------------------------------------------------------------
  const bool in_offboard_phase =
    (state_ != FlightState::WAIT_FOR_FCU) &&
    (state_ != FlightState::LANDING) &&
    (state_ != FlightState::DONE);

  if (in_offboard_phase) {
    publishOffboardControlMode();
  }

  switch (state_) {
    case FlightState::WAIT_FOR_FCU:     handleWaitForFcu();     break;
    case FlightState::STREAM_SETPOINT:  handleStreamSetpoint(); break;
    case FlightState::REQUEST_OFFBOARD: handleRequestOffboard();break;
    case FlightState::ARMING:           handleArming();         break;
    case FlightState::TAKEOFF:          handleTakeoff();        break;
    case FlightState::HOVER:            handleHover();          break;
    case FlightState::LANDING:          handleLanding();        break;
    case FlightState::DONE:                                     break;
  }
}

// -----------------------------------------------------------------------------
// 狀態 1：WAIT_FOR_FCU — 確認連線
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleWaitForFcu()
{
  // 判斷「連線正常」的三個條件：
  //   1. 有收到 VehicleLocalPosition   -> uXRCE-DDS 資料通道通了
  //   2. 有收到 VehicleStatus          -> 我們讀得到 arming/nav 狀態，後面才驗證得了
  //   3. z_valid == true               -> EKF2 的高度估計已經收斂，不是全 0 的垃圾值
  //
  // 第 3 點特別重要：Agent 剛連上時 PX4 就會開始發位置訊息，
  // 但那時 EKF2 還在初始化，z 是無效的。這時就切 Offboard 起飛非常危險。
  if (!local_position_received_ || !vehicle_status_received_) {
    // 每 2 秒提醒一次，不要洗版
    if (loop_count_ % (kLoopRateHz * 2) == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "仍在等待 PX4… (local_position=%s, vehicle_status=%s)",
                  local_position_received_ ? "OK" : "無",
                  vehicle_status_received_ ? "OK" : "無");
    }
    return;
  }

  if (!local_position_.z_valid) {
    if (loop_count_ % (kLoopRateHz * 2) == 0) {
      RCLCPP_WARN(this->get_logger(), "已連上 PX4，但 EKF2 高度估計尚未收斂，繼續等待…");
    }
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "PX4 連線正常！目前位置 NED = (%.2f, %.2f, %.2f)"
              "，相對 EKF 原點 %.2f m（原點不等於地面，僅供參考）",
              local_position_.x, local_position_.y, local_position_.z,
              -local_position_.z);

  transitionTo(FlightState::STREAM_SETPOINT, "開始預熱 setpoint 串流");
}

// -----------------------------------------------------------------------------
// 狀態 2：STREAM_SETPOINT — 發滿 1 秒心跳
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleStreamSetpoint()
{
  // 記住起飛點：把目前的水平位置與機頭方向鎖起來。
  // 這樣飛機是「原地垂直爬升」，而不是解鎖後衝向座標原點。
  if (!takeoff_origin_locked_) {
    takeoff_north_ = local_position_.x;
    takeoff_east_  = local_position_.y;
    // 連 z 一起鎖：這一刻飛機還在地上，所以這個 z 就是「地面」。
    // 少了這行，-takeoff_altitude_ 會被當成絕對目標，而 EKF 原點的偏移
    // 可能已經超過容忍值，導致程式判定「早就到了」而根本不爬升。
    takeoff_down_  = local_position_.z;
    takeoff_yaw_   = local_position_.heading;   // 保持目前機頭方向，不要無謂旋轉
    takeoff_origin_locked_ = true;
    RCLCPP_INFO(this->get_logger(),
                "鎖定起飛點：N=%.2f, E=%.2f, D=%.2f, yaw=%.1f°",
                takeoff_north_, takeoff_east_, takeoff_down_,
                takeoff_yaw_ * 180.0f / M_PI);
  }

  // 預熱階段的 setpoint 就是「停在原地、原本的高度」，
  // 因為此時還沒 Arm，PX4 只是在確認我們的串流有沒有穩定，不會真的動。
  publishTrajectorySetpoint(takeoff_north_, takeoff_east_,
                            takeoff_down_, takeoff_yaw_);

  if (loop_count_ >= kWarmupLoops) {
    RCLCPP_INFO(this->get_logger(), "已連續發送 %d 次 setpoint（約 1 秒），可以切模式了",
                loop_count_);
    transitionTo(FlightState::REQUEST_OFFBOARD, "要求切換 Offboard");
  }
}

// -----------------------------------------------------------------------------
// 狀態 3：REQUEST_OFFBOARD — 切模式並驗證
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleRequestOffboard()
{
  // 心跳持續（尚未起飛，維持原地原高度）
  publishTrajectorySetpoint(takeoff_north_, takeoff_east_,
                            takeoff_down_, takeoff_yaw_);

  // 先檢查是否已經切成功。
  // nav_state == NAVIGATION_STATE_OFFBOARD (=14) 才算數。
  if (vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    RCLCPP_INFO(this->get_logger(), "PX4 已進入 Offboard 模式 (nav_state=%d)",
                vehicle_status_.nav_state);
    // 記錄「確實進去過」。之後 nav_state 再離開 Offboard 就是飛手接管。
    offboard_confirmed_ = true;
    transitionTo(FlightState::ARMING, "準備 Arm");
    return;
  }

  // 還沒切成功 -> 每 0.5 秒重送一次指令。
  //
  // VEHICLE_CMD_DO_SET_MODE (=176) 的參數意義（MAVLink 標準）：
  //   param1 = base mode，填 1 代表 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
  //            （告訴 PX4：接下來我要指定「自訂模式」）
  //   param2 = PX4 custom main mode，6 就是 PX4_CUSTOM_MAIN_MODE_OFFBOARD
  // 所以 (1, 6) 這組魔術數字就是「切到 Offboard」。
  if ((loop_count_ - 1) % kRetryLoops == 0) {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    ++command_retry_count_;
    RCLCPP_INFO(this->get_logger(), "送出切換 Offboard 指令（第 %d 次），目前 nav_state=%d",
                command_retry_count_, vehicle_status_.nav_state);
  }

  if (command_retry_count_ > kMaxRetries) {
    RCLCPP_ERROR(this->get_logger(),
                 "切換 Offboard 失敗（已重試 %d 次）。常見原因："
                 "setpoint 串流不穩、PX4 預檢未通過、或 target_system(%d) 填錯。",
                 command_retry_count_, target_system_);
    transitionTo(FlightState::DONE, "切模式逾時，中止");
  }
}

// -----------------------------------------------------------------------------
// 狀態 4：ARMING — 解鎖並驗證
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleArming()
{
  publishTrajectorySetpoint(takeoff_north_, takeoff_east_,
                            takeoff_down_, takeoff_yaw_);

  // arming_state == ARMING_STATE_ARMED (=2) 代表馬達真的解鎖了
  if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
    RCLCPP_INFO(this->get_logger(), "飛機已 ARM！開始起飛");
    transitionTo(FlightState::TAKEOFF, "爬升中");
    return;
  }

  // VEHICLE_CMD_COMPONENT_ARM_DISARM (=400)：param1 = 1 解鎖、0 上鎖
  if ((loop_count_ - 1) % kRetryLoops == 0) {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    ++command_retry_count_;
    RCLCPP_INFO(this->get_logger(), "送出 ARM 指令（第 %d 次），目前 arming_state=%d",
                command_retry_count_, vehicle_status_.arming_state);
  }

  if (command_retry_count_ > kMaxRetries) {
    RCLCPP_ERROR(this->get_logger(),
                 "ARM 失敗（已重試 %d 次）。看一下 PX4 終端機的錯誤訊息，"
                 "常見是預檢沒過（pre_flight_checks_pass=%s）。",
                 command_retry_count_,
                 vehicle_status_.pre_flight_checks_pass ? "true" : "false");
    transitionTo(FlightState::DONE, "ARM 逾時，中止");
  }
}

// -----------------------------------------------------------------------------
// 狀態 5：TAKEOFF — 爬升到目標高度
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleTakeoff()
{
  // NED 往下為正，所以「從起飛點往上爬 h 公尺」= 起飛點的 z 再減去 h
  const float target_down = takeoff_down_ - static_cast<float>(takeoff_altitude_);

  publishTrajectorySetpoint(takeoff_north_, takeoff_east_, target_down, takeoff_yaw_);

  // 目前相對起飛點的高度 = 起飛點的 z 減去現在的 z（z 往下為正）
  const float current_altitude = takeoff_down_ - local_position_.z;
  const float altitude_error = std::fabs(current_altitude -
                                         static_cast<float>(takeoff_altitude_));

  // 每秒回報一次爬升進度
  if (loop_count_ % kLoopRateHz == 0) {
    RCLCPP_INFO(this->get_logger(), "爬升中… 目前高度 %.2f m / 目標 %.2f m（誤差 %.2f m）",
                current_altitude, takeoff_altitude_, altitude_error);
  }

  // 到達判定：高度誤差夠小，而且垂直速度也夠慢（代表真的穩住了，不是路過）
  const bool altitude_reached = altitude_error < static_cast<float>(position_tolerance_);
  const bool is_settled = std::fabs(local_position_.vz) < 0.3f;

  // 條件必須「連續」成立 kSettledLoops 次才算數。
  // 中間只要有任何一個週期不符合，計數就歸零重來 —— 這樣才能濾掉
  // 爬升減速過程中的瞬間擦邊，確保飛機是真的停在目標高度上。
  if (altitude_reached && is_settled) {
    ++settled_count_;
  } else {
    settled_count_ = 0;
  }

  if (settled_count_ >= kSettledLoops) {
    RCLCPP_INFO(this->get_logger(), "已穩定到達目標高度 %.2f m（連續 %d 個週期達標），開始懸停 %.1f 秒",
                current_altitude, settled_count_, hover_duration_);
    transitionTo(FlightState::HOVER, "懸停");
  }
}

// -----------------------------------------------------------------------------
// 狀態 6：HOVER — 懸停計時
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleHover()
{
  const float target_down = takeoff_down_ - static_cast<float>(takeoff_altitude_);

  // 懸停 = 持續送同一個位置 setpoint。心跳一停就會掉出 Offboard，所以不能偷懶。
  publishTrajectorySetpoint(takeoff_north_, takeoff_east_, target_down, takeoff_yaw_);

  const int hover_loops = static_cast<int>(hover_duration_ * kLoopRateHz);

  if (loop_count_ % kLoopRateHz == 0) {
    RCLCPP_INFO(this->get_logger(), "懸停中… %d / %.0f 秒（高度 %.2f m）",
                loop_count_ / kLoopRateHz, hover_duration_,
                takeoff_down_ - local_position_.z);
  }

  if (loop_count_ >= hover_loops) {
    RCLCPP_INFO(this->get_logger(), "懸停時間到，開始降落");
    transitionTo(FlightState::LANDING, "自動降落");
    // 進入 LANDING 的第一個 tick 就會送出降落指令
  }
}

// -----------------------------------------------------------------------------
// 狀態 7：LANDING — 交給 PX4 自動降落
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::handleLanding()
{
  // 這裡我們不再自己算 setpoint，而是送 VEHICLE_CMD_NAV_LAND (=21)，
  // 讓 PX4 切到內建的 AUTO_LAND 模式自己降。
  //
  // 好處：PX4 的降落有完整的著陸偵測（land detector），
  //       落地後會依 COM_DISARM_LAND 參數自動 disarm，
  //       流程跟真機完全一致。自己在 Offboard 裡慢慢降 z 的話，
  //       落地偵測要自己寫，很容易在觸地瞬間出狀況。
  //
  // 注意：上面 controlLoop() 已經在 LANDING 狀態停發 Offboard 心跳了，
  //       否則會跟 AUTO_LAND 搶控制權。
  if (loop_count_ == 1 || (loop_count_ - 1) % (kLoopRateHz * 2) == 0) {
    if (vehicle_status_.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LAND &&
        vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED)
    {
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      RCLCPP_INFO(this->get_logger(), "送出降落指令 NAV_LAND，目前 nav_state=%d",
                  vehicle_status_.nav_state);
    }
  }

  if (loop_count_ % kLoopRateHz == 0) {
    RCLCPP_INFO(this->get_logger(), "降落中… 高度 %.2f m，nav_state=%d，arming_state=%d",
                takeoff_down_ - local_position_.z, vehicle_status_.nav_state,
                vehicle_status_.arming_state);
  }

  // 完成判定：PX4 自己 disarm 了（arming_state 不再是 ARMED）
  if (vehicle_status_.arming_state != VehicleStatus::ARMING_STATE_ARMED) {
    RCLCPP_INFO(this->get_logger(), "已降落並自動 DISARM，任務完成");
    transitionTo(FlightState::DONE, "任務結束");
  }

  // 保險：降落超過 60 秒還沒 disarm，主動送一次 disarm
  if (loop_count_ > kLoopRateHz * 60) {
    RCLCPP_WARN(this->get_logger(), "降落逾時，主動送出 DISARM");
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    transitionTo(FlightState::DONE, "逾時強制結束");
  }
}

// -----------------------------------------------------------------------------
// 發送：OffboardControlMode（心跳）
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::publishOffboardControlMode()
{
  OffboardControlMode msg{};

  // 這六個布林值是「互斥的控制層級」，只能有一個為 true。
  // PX4 依照 position -> velocity -> acceleration -> attitude -> body_rate
  // 的優先序，挑第一個 true 的當作控制輸入。
  // 我們要做位置控制起飛，所以只開 position。
  msg.position     = true;
  msg.velocity     = false;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;

  msg.timestamp = nowMicros();
  offboard_mode_pub_->publish(msg);
}

// -----------------------------------------------------------------------------
// 發送：TrajectorySetpoint（位置目標）
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::publishTrajectorySetpoint(
  float north, float east, float down, float yaw)
{
  TrajectorySetpoint msg{};

  // position 是 float32[3]，順序就是 NED：[0]=North, [1]=East, [2]=Down
  // 再次提醒：down 是負值代表往上飛。
  msg.position = {north, east, down};

  // yaw 單位是弧度，範圍 -PI..+PI，以正北為 0、順時針為正。
  msg.yaw = yaw;

  // 其他欄位（velocity / acceleration / jerk / yawspeed）在 PX4 v1.14 的預設值是 0。
  // 若要「不控制」某一項，PX4 的慣例是填 NaN。這裡做純位置控制，維持預設即可。

  msg.timestamp = nowMicros();
  setpoint_pub_->publish(msg);
}

// -----------------------------------------------------------------------------
// 發送：VehicleCommand（切模式 / Arm / Land …）
// -----------------------------------------------------------------------------
void OffboardTakeoffNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};

  msg.command = command;
  msg.param1  = param1;
  msg.param2  = param2;

  // target_system：這道指令要給哪一台飛機。
  // 單機 SITL 是 1；多機時第 N 台（instance N）是 N+1。
  // 填錯的話指令會被忽略 —— 三機編隊時這是最容易出錯的一格。
  msg.target_system    = static_cast<uint8_t>(target_system_);
  msg.target_component = 1;    // 1 = autopilot 元件

  // source_*：宣告「這道指令是誰發的」。用 1 是沿用 PX4 官方範例的慣例。
  msg.source_system    = 1;
  msg.source_component = 1;

  // from_external = true 代表「來自機外的地面站/伴飛電腦」。
  // 這個一定要填 true，否則 PX4 會把它當成內部指令而拒絕執行。
  msg.from_external = true;

  msg.timestamp = nowMicros();
  command_pub_->publish(msg);
}

// -----------------------------------------------------------------------------
// 工具函式
// -----------------------------------------------------------------------------
uint64_t OffboardTakeoffNode::nowMicros()
{
  // PX4 所有訊息的 timestamp 欄位單位都是「微秒」。
  // ROS 2 的 now() 是奈秒，所以要除以 1000。
  // uXRCE-DDS client 會自動把這個時間戳換算成 PX4 內部時間，所以直接用 ROS 時鐘沒問題。
  return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
}

void OffboardTakeoffNode::transitionTo(FlightState next, const std::string & reason)
{
  RCLCPP_INFO(this->get_logger(), "[狀態] %s -> %s : %s",
              stateName(state_), stateName(next), reason.c_str());
  state_ = next;
  loop_count_ = 0;            // 每進新狀態就重新計時
  command_retry_count_ = 0;
  settled_count_ = 0;
}

const char * OffboardTakeoffNode::stateName(FlightState s)
{
  switch (s) {
    case FlightState::WAIT_FOR_FCU:     return "WAIT_FOR_FCU";
    case FlightState::STREAM_SETPOINT:  return "STREAM_SETPOINT";
    case FlightState::REQUEST_OFFBOARD: return "REQUEST_OFFBOARD";
    case FlightState::ARMING:           return "ARMING";
    case FlightState::TAKEOFF:          return "TAKEOFF";
    case FlightState::HOVER:            return "HOVER";
    case FlightState::LANDING:          return "LANDING";
    case FlightState::DONE:             return "DONE";
  }
  return "UNKNOWN";
}

}  // namespace drone_control

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  // 讓 printf/RCLCPP 的輸出不要被緩衝住，這樣你在終端機能即時看到流程。
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_control::OffboardTakeoffNode>());
  rclcpp::shutdown();
  return 0;
}
