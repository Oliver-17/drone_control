// =============================================================================
//  offboard_takeoff_node.hpp
//
//  PX4 v1.14 + ROS 2 Humble：單機 Offboard 起飛 / 懸停 / 降落節點
//
//  這支程式的完整流程（狀態機）：
//     WAIT_FOR_FCU     等 PX4 傳位置資料過來，確認 uXRCE-DDS 連線正常
//  -> STREAM_SETPOINT  以 10Hz 發 setpoint 心跳，發滿 1 秒
//  -> REQUEST_OFFBOARD 要求 PX4 切到 Offboard 模式，並「驗證」真的切成功
//  -> ARMING           送 Arm 指令，並「驗證」真的 armed
//  -> TAKEOFF          爬升到目標高度
//  -> HOVER            懸停 N 秒
//  -> LANDING          送 NAV_LAND 交給 PX4 自動降落
//  -> DONE             完成
//
//  設計成 class 放在 header，是為了之後做三機編隊時，
//  可以直接 include 這個檔案、把它當成「單機控制器」的基底來擴充。
// =============================================================================

#ifndef DRONE_CONTROL__OFFBOARD_TAKEOFF_NODE_HPP_
#define DRONE_CONTROL__OFFBOARD_TAKEOFF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <string>

namespace drone_control
{

/**
 * @brief 飛行流程的狀態機。
 *
 * 為什麼用 enum 狀態機，而不是像 PX4 官方範例那樣用一個 counter 數數？
 *   官方 offboard_control.cpp 的做法是「counter 數到 10 就切模式+Arm」，
 *   完全不檢查有沒有成功。單機 demo 可以，但是：
 *     1. 切模式失敗時你不會知道（畫面上什麼都沒有）
 *     2. 沒有降落流程
 *     3. 之後三台飛機各自進度不同，用 counter 根本管不動
 *   用具名狀態機的話，每台飛機都有明確的「我現在在哪一步」，
 *   之後編隊只要加一個 WAIT_FOR_ALL（等其他兩台也到 HOVER）就好。
 */
enum class FlightState
{
  WAIT_FOR_FCU,       ///< 等待 PX4 連線（收到有效的 local position）
  STREAM_SETPOINT,    ///< 預先發送 setpoint 心跳，累積滿 1 秒
  REQUEST_OFFBOARD,   ///< 送出切換 Offboard 模式的指令，並等待生效
  ARMING,             ///< 送出 Arm 指令，並等待馬達真的解鎖
  TAKEOFF,            ///< 爬升到目標高度
  HOVER,              ///< 懸停計時
  LANDING,            ///< 自動降落中
  DONE                ///< 全部完成
};

class OffboardTakeoffNode : public rclcpp::Node
{
public:
  OffboardTakeoffNode();

private:
  // ---------------------------------------------------------------------------
  // 主迴圈
  // ---------------------------------------------------------------------------

  /// 10Hz 定時器回呼，狀態機的心臟。
  void controlLoop();

  // ---------------------------------------------------------------------------
  // 各狀態的處理函式（每個對應狀態機裡的一格）
  // ---------------------------------------------------------------------------
  void handleWaitForFcu();
  void handleStreamSetpoint();
  void handleRequestOffboard();
  void handleArming();
  void handleTakeoff();
  void handleHover();
  void handleLanding();

  // ---------------------------------------------------------------------------
  // 發送訊息給 PX4
  // ---------------------------------------------------------------------------

  /// 發送 OffboardControlMode「心跳」，告訴 PX4 我們要用哪一層控制（這裡是位置控制）。
  void publishOffboardControlMode();

  /// 發送位置 setpoint（NED 座標，單位公尺）。
  void publishTrajectorySetpoint(float north, float east, float down, float yaw);

  /// 發送 VehicleCommand（切模式 / Arm / Disarm / Land 都走這個）。
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

  // ---------------------------------------------------------------------------
  // 訂閱回呼
  // ---------------------------------------------------------------------------
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // 小工具
  // ---------------------------------------------------------------------------

  /// PX4 的所有訊息 timestamp 都是「微秒」，而且必須填，否則 PX4 會忽略該訊息。
  uint64_t nowMicros();

  /// 切換狀態並印一行 log（方便你在終端機上追流程）。
  void transitionTo(FlightState next, const std::string & reason);

  static const char * stateName(FlightState s);

  // ---------------------------------------------------------------------------
  // 參數（多機擴充的關鍵：所有東西都可以從外面設定，程式碼不用改）
  // ---------------------------------------------------------------------------
  std::string vehicle_name_;     ///< 這台飛機的名字，例如 "MAV1"。只影響 log 與識別，不影響 topic
  std::string px4_namespace_;    ///< "" 或 "/MAV1"、"/MAV2"…，必須與 PX4 的 PX4_UXRCE_DDS_NS 一致
  int         target_system_;    ///< MAVLink system id：單機=1，多機= instance+1
  double      takeoff_altitude_; ///< 起飛高度（正值，公尺。程式內部會轉成 NED 的負值）
  double      hover_duration_;   ///< 懸停秒數
  double      position_tolerance_; ///< 判定「到達目標高度」的誤差容忍（公尺）

  // ---------------------------------------------------------------------------
  // ROS 介面
  // ---------------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr       vehicle_status_sub_;

  // ---------------------------------------------------------------------------
  // 內部狀態
  // ---------------------------------------------------------------------------
  FlightState state_{FlightState::WAIT_FOR_FCU};

  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleStatus        vehicle_status_{};

  bool local_position_received_{false};
  bool vehicle_status_received_{false};

  int  loop_count_{0};            ///< 進入目前狀態後，迴圈跑了幾次（10 次 = 1 秒）
  int  command_retry_count_{0};   ///< 指令重送次數（切模式 / Arm 沒生效時會重送）
  int  settled_count_{0};         ///< 「到達目標高度」的條件已連續成立幾個週期

  // 起飛時鎖定的水平位置與機頭方向。
  // 不寫死 (0,0)，是因為之後三機各自停在不同起點，
  // 每台都應該「在自己的正上方」垂直爬升，而不是全部飛到原點。
  float takeoff_north_{0.0f};
  float takeoff_east_{0.0f};
  // 起飛當下的 NED z，等同於「地面」在 EKF 座標系裡的位置。
  // 為什麼要存：z=0 是 EKF 的原點，不保證等於地面。實測 SITL 剛開機時
  // 飛機明明停在地上，z 卻是 -0.42。所有高度都改用「相對這個點」計算，
  // 才不會被原點偏移騙到（真機上偏移可能更大）。
  float takeoff_down_{0.0f};
  float takeoff_yaw_{0.0f};
  bool  takeoff_origin_locked_{false};
};

}  // namespace drone_control

#endif  // DRONE_CONTROL__OFFBOARD_TAKEOFF_NODE_HPP_
