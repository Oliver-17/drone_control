// =============================================================================
//  cmd_vel_to_px4_node —— 把 ROS 的速度指令翻譯成 PX4 的 setpoint（階段 2 / S2）
//
//  它做什麼：
//      訂 <ns>/cmd_vel                      （geometry_msgs/Twist，機體座標系）
//      發 /fmu/in/trajectory_setpoint       （NED 世界座標系）
//      發 /fmu/in/offboard_control_mode     （不發的話 PX4 會掉出 offboard）
//
//  跟 px4_tf_node 的關係：同一條轉換鏈的正反向。
//      px4_tf_node          PX4 -> ROS   「我在哪」   感測
//      cmd_vel_to_px4_node  ROS  -> PX4  「往那邊走」  控制
//
//  ⚠️ 它是第四個會發 trajectory_setpoint 的節點（另外三個是 offboard_takeoff_node、
//     fly_nodes.py、precision_land_node）。PX4 只認一條串流，兩個同時發會讓飛機抽搐，
//     而且在 log 裡看起來只像控制器沒調好。
//     所以這支「沒收到 cmd_vel 就完全不發布」—— 不是發零速度，是連 publisher 都收掉，
//     這樣別人的 count_publishers 檢查才擋得住。
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;

class CmdVelToPx4Node : public rclcpp::Node
{
public:
  CmdVelToPx4Node()
  : rclcpp::Node("cmd_vel_to_px4_node")
  {
    px4_namespace_ = declare_parameter<std::string>("px4_namespace", "MAV1");
    // 定高飛行的目標高度（公尺，離地正值）。Nav2 只管平面，高度由這裡寫死。
    flight_altitude_ = declare_parameter<double>("flight_altitude", 3.0);
    // 超過這個時間沒收到 cmd_vel 就視為「沒人在控制」，停止發布。
    // 0.5 秒的依據：Nav2 controller 預設 20 Hz，掉十幀才觸發，不會誤判。
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);
    max_speed_xy_ = declare_parameter<double>("max_speed_xy", 1.5);
    // 定高用的 P 增益與速度上限。不要設 1.0 —— 一次修到位會過衝然後來回震盪。
    kp_z_ = declare_parameter<double>("kp_z", 0.6);
    max_climb_speed_ = declare_parameter<double>("max_climb_speed", 1.0);
    max_yawspeed_ = declare_parameter<double>("max_yawspeed", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    target_system_ = declare_parameter<int>("target_system", 1);

    rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
    sub_qos.best_effort().durability_volatile();
    // ROS -> PX4 的指令用預設 reliable，相容性最好
    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));

    const std::string ns = "/" + px4_namespace_;

    // 需要 heading 才能把「機體速度」轉成「世界速度」
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ns + "/fmu/out/vehicle_local_position_v1", sub_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr m) {
        local_position_ = *m;
        has_position_ = true;
      });

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      ns + "/fmu/out/vehicle_status_v1", sub_qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr m) {vehicle_status_ = *m;});

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&CmdVelToPx4Node::onCmdVel, this, std::placeholders::_1));

    // ⚠️ 這兩個 publisher 刻意「用到才建、逾時就銷毀」。
    //    只停止送訊息是不夠的：Publisher 物件還在的話，別人用
    //    count_publishers("trajectory_setpoint") 檢查「還有沒有人在控制」時，
    //    仍然會數到我們，於是拒絕接手 —— 交接就卡死了。
    //    （drone_apriltag_landing 的 precision_land_node 就是這樣檢查的。）
    //    代價是重新建立時 DDS discovery 要約 1 秒，但接手不是時間關鍵的動作，
    //    那段期間 PX4 會維持 HOLD 懸停，安全。
    pub_qos_ = pub_qos;
    setpoint_topic_ = ns + "/fmu/in/trajectory_setpoint";
    offboard_mode_topic_ = ns + "/fmu/in/offboard_control_mode";
    // 切模式的指令用一般的 publisher，一直存在沒關係 ——
    // 它不是 setpoint 串流，別人的 count_publishers 檢查不會看它。
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      ns + "/fmu/in/vehicle_command", pub_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelToPx4Node::loop, this));

    logStartup();
  }

private:
  void logStartup()
  {
    RCLCPP_INFO(get_logger(), "===== cmd_vel_to_px4_node 啟動（骨架版）=====");
    RCLCPP_INFO(get_logger(), "  px4_namespace   : %s", px4_namespace_.c_str());
    RCLCPP_INFO(get_logger(), "  flight_altitude : %.2f m（Nav2 不管高度，由這裡鎖）",
      flight_altitude_);
    RCLCPP_INFO(get_logger(), "  cmd_timeout     : %.2f s（逾時就停止發布）",
      cmd_timeout_s_);
    RCLCPP_INFO(get_logger(), "  發布頻率        : %.0f Hz", publish_rate_hz_);
    RCLCPP_INFO(get_logger(), "  等待 cmd_vel…（沒收到就完全不發布，不搶控制權）");
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = now();
    has_cmd_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "收到 cmd_vel：前 %+.2f 左 %+.2f 轉 %+.2f",
      msg->linear.x, msg->linear.y, msg->angular.z);
  }

  // ---------------------------------------------------------------------------
  //  20 Hz 主迴圈
  // ---------------------------------------------------------------------------
  void loop()
  {
    if (!has_cmd_ || !has_position_) {return;}

    // 逾時就「完全停止發布」，而不是發零速度。
    // 差別很重要：發零速度的話這個 publisher 一直存在，別人（例如降落節點）
    // 用 count_publishers 檢查「還有沒有人在控制」時會誤判成有，直接拒絕接手。
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_s_) {
      if (was_publishing_) {
        was_publishing_ = false;
        // 真的把 publisher 收掉，count_publishers 才會歸零
        setpoint_pub_.reset();
        offboard_mode_pub_.reset();
        RCLCPP_INFO(get_logger(),
          "超過 %.2f 秒沒收到 cmd_vel，已銷毀 publisher，交還控制權",
          cmd_timeout_s_);
      }
      return;
    }
    if (!was_publishing_) {
      was_publishing_ = true;
      offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        offboard_mode_topic_, pub_qos_);
      setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        setpoint_topic_, pub_qos_);
      // 接手的瞬間把航向目標對齊當下機頭，否則會從上次的舊值猛轉回去
      yaw_setpoint_ = local_position_.heading;
      RCLCPP_INFO(get_logger(),
        "收到 cmd_vel，開始接管（定高 %.2f m，航向鎖在 %.1f°）",
        flight_altitude_, yaw_setpoint_ * 180.0 / M_PI);
    }

    publishOffboardControlMode();
    publishSetpoint();
    ensureOffboard();
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode m{};
    // 這幾個布林是「互斥的控制層級」，只能有一個為 true。
    //
    // ⚠️ 這裡曾經改過一次：原本用「水平速度 + 垂直位置」的混合設定
    //    （position=true、sp.position[2] 給目標高度、sp.velocity[0..1] 給水平），
    //    語法上合法（符合 PositionControl.cpp:229-235 的成對規則），
    //    但實測起飛後高度會振盪 ±3 m，要三十秒才收斂。
    //    改成「全速度控制、高度用自己的 P 控制器」之後就穩了 ——
    //    這也是 drone_apriltag_landing 的 precision_land_node 用的做法，
    //    那支在 T3 實飛驗證過。
    m.position = false;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.timestamp = nowMicros();
    offboard_mode_pub_->publish(m);
  }

  void publishSetpoint()
  {
    // --- 機體 FLU -> 世界 NED ---
    // Nav2 的 cmd_vel 是「機體座標系」，而且用 ROS 慣例 FLU（x 前 / y 左）。
    // PX4 要的是世界 NED。轉兩步：
    //   ① FLU -> FRD：y 反向（左變右）
    //   ② FRD -> NED：繞「下」軸轉 heading
    // 這條鏈和 drone_apriltag_landing 的 precision_land_node 用的是同一條，
    // 只是方向相反 —— 那邊已經在 Gazebo 實測驗證過。
    const double vx_frd = last_cmd_.linear.x;
    const double vy_frd = -last_cmd_.linear.y;
    const double h = local_position_.heading;

    double v_north = vx_frd * std::cos(h) - vy_frd * std::sin(h);
    double v_east = vx_frd * std::sin(h) + vy_frd * std::cos(h);

    // 安全上限。Nav2 的參數理論上已經限速，但這裡再夾一次 ——
    // 參數填錯或別人誤發一個大數值時，這是最後一道防線。
    const double speed = std::hypot(v_north, v_east);
    if (speed > max_speed_xy_ && speed > 1e-6) {
      const double k = max_speed_xy_ / speed;
      v_north *= k;
      v_east *= k;
    }

    // ROS 的 angular.z 是「繞上軸、逆時針為正」，
    // PX4 的 yawspeed 是「繞下軸、順時針為正」—— 方向相反，要變號。
    const double yawspeed = clampAbs(-last_cmd_.angular.z, max_yawspeed_);

    // ⚠️ 只送 yawspeed、把 yaw 留 NaN 是不行的。
    //    PX4 的位置控制器在 yaw 為 NaN 時，會每一輪把「目標航向」設成「當下航向」：
    //        PositionControl.cpp:117
    //        _yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
    //    等於完全沒有回正力 —— 任何擾動都會累積，飛機會慢慢自轉，
    //    而且轉到哪裡都「沒有錯」，所以不會有任何警告。
    //    （PX4 自己在那行留了 TODO，可見這是已知的粗糙處。）
    //
    //    所以這裡自己積分出一個絕對航向目標：不轉的時候它是固定值 = 真正的定向，
    //    要轉的時候才跟著動。yawspeed 仍然照送，當作前饋讓轉動跟手。
    const double dt = 1.0 / publish_rate_hz_;
    yaw_setpoint_ = wrapPi(yaw_setpoint_ + yawspeed * dt);

    // 垂直：用 P 控制器把高度拉回目標，而不是給位置 setpoint。
    // NED 的 z 是「下為正」，所以離地高度 = -z。
    // err > 0 代表飛太高，要往下（vz 正）。
    const double alt_err = (-local_position_.z) - flight_altitude_;
    const double v_down = clampAbs(kp_z_ * alt_err, max_climb_speed_);

    px4_msgs::msg::TrajectorySetpoint sp{};
    const float nan = std::numeric_limits<float>::quiet_NaN();

    // 位置全給 NaN 代表「這一層不控制」，PX4 才會走速度控制。
    // 三軸都用速度，符合 PositionControl.cpp:229-235 的規則
    //（每軸至少一種 setpoint、x 與 y 成對）。
    // 這就是「定高飛行」的實作方式：Nav2 只管平面，高度由本節點的 P 控制器鎖住。
    sp.position = {nan, nan, nan};
    sp.velocity = {static_cast<float>(v_north), static_cast<float>(v_east),
      static_cast<float>(v_down)};
    sp.acceleration = {nan, nan, nan};
    sp.jerk = {nan, nan, nan};
    sp.yaw = static_cast<float>(yaw_setpoint_);    // 絕對航向，維持定向用
    sp.yawspeed = static_cast<float>(yawspeed);    // 角速度前饋
    sp.timestamp = nowMicros();
    setpoint_pub_->publish(sp);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "cmd_vel(前%+.2f 左%+.2f 轉%+.2f) -> NED(北%+.2f 東%+.2f 下%+.2f)  "
      "高度 %.2f/%.2f m  航向 %.0f°",
      last_cmd_.linear.x, last_cmd_.linear.y, last_cmd_.angular.z,
      v_north, v_east, v_down,
      -local_position_.z, flight_altitude_, yaw_setpoint_ * 180.0 / M_PI);
  }

  // 確保飛機在 offboard 模式。
  //
  // 為什麼需要：這支是「有人送 cmd_vel 才接管」的設計，而在那之前 PX4
  // 可能掉在 HOLD（例如起飛工具退場之後）。只發 setpoint 是不會自動切模式的，
  // 症狀會是「Nav2 有規劃、cmd_vel 有在發，但飛機一動也不動」。
  //
  // 只在「已經有人在送 cmd_vel」時才切 —— 沒人要控制的時候不去碰飛機。
  void ensureOffboard()
  {
    constexpr uint8_t kOffboard =
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (vehicle_status_.nav_state == kOffboard) {
      offboard_retry_ = 0;
      return;
    }
    // 每 0.5 秒重送一次（20 Hz 迴圈 = 每 10 次）。
    // DO_SET_MODE 的 (param1=1, param2=6) 就是「切到 Offboard」。
    if (offboard_retry_ % 10 == 0) {
      px4_msgs::msg::VehicleCommand m{};
      m.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
      m.param1 = 1.0f;
      m.param2 = 6.0f;
      m.target_system = static_cast<uint8_t>(target_system_);
      m.target_component = 1;
      m.source_system = 1;
      m.source_component = 1;
      m.from_external = true;   // 少了這個 PX4 會當成內部指令直接拒絕
      m.timestamp = nowMicros();
      command_pub_->publish(m);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "有人在送 cmd_vel 但飛機不在 offboard（nav_state=%d），送出切換指令",
        vehicle_status_.nav_state);
    }
    ++offboard_retry_;
  }

  static double clampAbs(double v, double lim)
  {
    return std::max(-lim, std::min(lim, v));
  }

  static double wrapPi(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  uint64_t nowMicros() {return static_cast<uint64_t>(now().nanoseconds() / 1000);}

  std::string px4_namespace_;
  double flight_altitude_{3.0}, cmd_timeout_s_{0.5};
  double max_speed_xy_{1.5}, max_yawspeed_{1.0}, publish_rate_hz_{20.0};
  double kp_z_{0.6}, max_climb_speed_{1.0};
  int target_system_{1}, offboard_retry_{0};
  px4_msgs::msg::VehicleStatus vehicle_status_;
  bool has_position_{false}, has_cmd_{false}, was_publishing_{false};
  double yaw_setpoint_{0.0};   // 自己維護的絕對航向目標（弧度，NED）

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  px4_msgs::msg::VehicleLocalPosition local_position_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::QoS pub_qos_{rclcpp::KeepLast(10)};
  std::string setpoint_topic_, offboard_mode_topic_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToPx4Node>());
  rclcpp::shutdown();
  return 0;
}
