// =============================================================================
//  px4_tf_node —— 把 PX4 的位置與姿態翻譯成 ROS 的 TF（階段 1 / S1）
//
//  它做什麼：
//      訂 /fmu/out/vehicle_local_position_v1（NED 位置）
//      訂 /fmu/out/vehicle_attitude        （FRD->NED 四元數）
//      發 TF: map -> odom -> base_link
//
//  為什麼需要它：
//      整個 ROS 世界（Nav2、RViz、AprilTag 降落）都靠 TF 知道「機器人在哪、面向哪」，
//      但 PX4 不發 TF，它只發自己格式的訊息，而且用的是完全不同的座標慣例：
//          PX4：NED（北-東-下）+ FRD 機體（前-右-下）
//          ROS：ENU（東-北-上）+ FLU 機體（前-左-上）
//      少了這個翻譯，Nav2 會安靜地不動，RViz 會顯示 "Fixed Frame [map] does not exist"。
//
//  為什麼是 map -> odom -> base_link 三層而不是直接 map -> base_link：
//      TF 是一棵樹，每個 frame 只能有一個父節點。之後接上 SLAM 時，SLAM 要發布
//      「修正量」，它只能插在 map -> odom 這一段，不能搶走 base_link 的父節點。
//      現在先把架構留對，之後把 publish_map_to_odom 關掉就能無縫接上 SLAM。
//
//  ⚠️ 這支最容易錯的地方是座標轉換的方向與符號。錯了不會報錯，
//     只會讓 Nav2 很認真地往錯的方向導航。所以配了 test/t_tf_check.py。
//
//  骨架階段：參數與訂閱/發布都已就位並可執行，但轉換還沒實作。
// =============================================================================

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

namespace
{

// -----------------------------------------------------------------------------
//  座標轉換的兩個常數旋轉
//
//  來源：PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp:934-949
//  的 GZBridge::rotateQuaternion()。那是 PX4 官方在做 Gazebo(ROS 慣例) 與 PX4
//  之間轉換的實作，直接沿用它的定義，不自己推導。
//
//  ⚠️ 注意四元數的分量順序：
//      PX4 的 q[4]            是 (w, x, y, z)
//      tf2::Quaternion 建構子  是 (x, y, z, w)
//      ROS geometry_msgs      是 (x, y, z, w)
//  順序填反是這裡最常見的錯誤，而且不會報錯，只會讓姿態整個亂掉。
// -----------------------------------------------------------------------------

// FLU(ROS 機體) <-> FRD(PX4 機體)：繞 X 轉 180 度。
// GZBridge.cpp:937 寫成 Quaterniond(0, 1, 0, 0)，那是 (w,x,y,z) 順序。
tf2::Quaternion qFluToFrd()
{
  return tf2::Quaternion(1.0, 0.0, 0.0, 0.0);   // (x, y, z, w)
}

// ENU <-> NED：先繞 Z 轉 +90 度，再繞新的 X 轉 180 度。
// GZBridge.cpp:946 寫成 Quaterniond(0, 0.70711, 0.70711, 0)，同樣是 (w,x,y,z)。
// 該處註解明講「This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU」，
// 所以正反兩個方向可以共用同一個值。
tf2::Quaternion qEnuToNed()
{
  return tf2::Quaternion(0.70710678, 0.70710678, 0.0, 0.0);   // (x, y, z, w)
}

}  // namespace

class Px4TfNode : public rclcpp::Node
{
public:
  Px4TfNode()
  : rclcpp::Node("px4_tf_node")
  {
    px4_namespace_ = declare_parameter<std::string>("px4_namespace", "MAV1");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // 之後接上 SLAM 時要改成 false —— 兩個節點同時發同一段 TF 會讓整棵樹閃爍，
    // 而症狀是「機器人位置一直跳」，非常難查。
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);

    // ⚠️ map 和 odom 不能假設重合。
    //
    //    odom 的原點 = PX4 的 EKF 初始化位置 = 飛機開機/起飛的地方。
    //    map 的原點 = 靜態地圖的原點 = Gazebo 世界原點。
    //    飛機只要不是 spawn 在世界原點，兩者就差一個固定偏移。
    //
    //    不設這個偏移的話，症狀是「costmap 裡的牆跟光達掃到的牆整體錯開」——
    //    而且所有 TF 看起來都很正常（因為它們彼此自洽），
    //    拿 TF 跟 PX4 的位置對照也永遠一致。只有跟「世界」比才看得出來。
    //    實測這個坑：飛機 spawn 在 (-2, 0)，光達的回波有一半落不到牆上。
    //
    //    單位是 ENU（x 東、y 北、z 上），yaw 是弧度。
    //    之後接 SLAM 時把 publish_map_to_odom 設成 false，改由 SLAM 發這一段。
    auto off = declare_parameter<std::vector<double>>(
      "odom_origin_in_map", {0.0, 0.0, 0.0});
    for (size_t i = 0; i < 3; ++i) {
      odom_origin_[i] = (off.size() > i) ? off[i] : 0.0;
    }
    odom_origin_yaw_ = declare_parameter<double>("odom_origin_yaw", 0.0);

    // ⚠️ 開機時自動對齊航向。
    //
    //    PX4 的航向來自 EKF2 融合磁力計的估計，而它認為的「北」和
    //    Gazebo 世界的北不一定一致（實測差 8 度：世界的磁場向量、
    //    PX4 依經緯度查表得到的磁偏角、EKF 的收斂狀態，三者湊出來的）。
    //    odom 的朝向是「PX4 的北」，map 的朝向是「世界的北」——
    //    差的那個角度就該放在 map->odom 裡。
    //
    //    不補的話：所有 TF 仍然彼此自洽，但 costmap 會把光達掃到的牆
    //    整體轉一個角度疊在靜態地圖上。實測 8 度在 16 m 處偏 2.2 m，
    //    足以讓規劃器以為走廊被堵住 —— 而且完全不會報錯。
    //
    //    打開這個之後，節點會等第一筆姿態，用
    //        map->odom 的 yaw = initial_map_yaw - (PX4 估出來的 ENU yaw)
    //    算出偏移。initial_map_yaw 是「開機時機頭在 map 座標的朝向」，
    //    由使用者告知（SITL 就是 spawn 的 yaw）。
    //
    //    之後接 SLAM 的話把這個關掉、publish_map_to_odom 也關掉，
    //    那段旋轉由 SLAM 負責。
    align_yaw_on_start_ = declare_parameter<bool>("align_yaw_on_start", false);
    initial_map_yaw_ = declare_parameter<double>("initial_map_yaw", 0.0);

    // PX4 -> ROS 的 topic 一律 best effort。用預設的 reliable 會完全收不到，
    // 而且不會有任何錯誤訊息，只是安靜地沒有資料。
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort().durability_volatile();

    const std::string ns = "/" + px4_namespace_;

    // 訊息版本化：MESSAGE_VERSION = 1 的 topic 才帶 _v1 後綴。
    // VehicleLocalPosition 是 1，VehicleAttitude 是 0。
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ns + "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&Px4TfNode::onLocalPosition, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      ns + "/fmu/out/vehicle_attitude", qos,
      std::bind(&Px4TfNode::onAttitude, this, std::placeholders::_1));

    start_time_ = now();
    // Nav2 生態（bt_navigator、velocity_smoother、部分 BT 節點）預期有 /odom。
    // 內容和 odom->base_link 這段 TF 完全一樣，只是換一種格式 ——
    // 有些元件讀 TF、有些讀 topic，兩邊都提供最省事。
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(10)));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // 需要自動對齊的話，要等第一筆姿態才算得出 yaw，所以延後發布
    if (publish_map_to_odom_ && !align_yaw_on_start_) {publishStaticMapToOdom();}
    logStartup();
  }

private:
  void logStartup()
  {
    RCLCPP_INFO(get_logger(), "===== px4_tf_node 啟動（骨架版）=====");
    RCLCPP_INFO(get_logger(), "  px4_namespace : %s", px4_namespace_.c_str());
    RCLCPP_INFO(get_logger(), "  TF 樹         : %s -> %s -> %s",
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  map->odom     : %s",
      publish_map_to_odom_ ? "由本節點發（單位變換）" : "交給別人發（例如 SLAM）");
    if (align_yaw_on_start_) {
      RCLCPP_INFO(get_logger(),
        "  航向自動對齊  : 開，開機朝向 %.1f°（等 %.0f 秒讓 EKF 收斂）",
        initial_map_yaw_ * 180.0 / M_PI, align_delay_s_);
    }
    RCLCPP_INFO(get_logger(), "  等待 PX4 的位置與姿態…");
  }

  // map -> odom 現階段是固定的偏移（不是單位變換）。
  //
  // 理由：模擬裡 PX4 的位置估計幾乎是真值、不會飄，所以這一段不需要動態修正；
  // 但 odom 的原點是「起飛點」，map 的原點是「世界原點」，
  // 兩者之間有一個固定的平移，要由 odom_origin_in_map 補上。
  //
  // 之後接 SLAM 時，改由 slam_toolbox 發這一段（把 publish_map_to_odom 設 false），
  // 下游的 odom -> base_link 完全不用改。
  void publishStaticMapToOdom()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform.translation.x = odom_origin_[0];
    t.transform.translation.y = odom_origin_[1];
    t.transform.translation.z = odom_origin_[2];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_origin_yaw_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    static_broadcaster_->sendTransform(t);
    static_sent_ = true;
    RCLCPP_INFO(get_logger(),
      "%s -> %s 固定偏移：ENU (%.2f, %.2f, %.2f) yaw %.1f°"
      "（odom 的原點就是飛機開機的位置，不一定是世界原點）",
      map_frame_.c_str(), odom_frame_.c_str(),
      odom_origin_[0], odom_origin_[1], odom_origin_[2],
      odom_origin_yaw_ * 180.0 / M_PI);
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    local_position_ = *msg;
    has_position_ = true;
    // 由位置訊息驅動發布：它的頻率（約 30 Hz）夠高，而姿態通常更快，
    // 兩者都用「最新收到的值」即可，不需要嚴格時間同步 ——
    // 這個等級的時間差（幾毫秒）對導航沒有影響。
    publishOdomToBase();
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    attitude_ = *msg;
    has_attitude_ = true;

    if (publish_map_to_odom_ && align_yaw_on_start_ && !static_sent_) {
      // 等 EKF 的航向收斂一點再取值。開機瞬間的估計會跳，
      // 拿第一筆就定案的話偏移會是錯的。
      if ((now() - start_time_).seconds() < align_delay_s_) {return;}
      const double px4_enu_yaw = currentEnuYaw();
      odom_origin_yaw_ = wrapPi(initial_map_yaw_ - px4_enu_yaw);
      RCLCPP_INFO(get_logger(),
        "自動對齊航向：PX4 估出的 ENU yaw %.1f°，指定的開機朝向 %.1f°，"
        "所以 %s->%s 轉 %.1f°",
        px4_enu_yaw * 180.0 / M_PI, initial_map_yaw_ * 180.0 / M_PI,
        map_frame_.c_str(), odom_frame_.c_str(),
        odom_origin_yaw_ * 180.0 / M_PI);
      publishStaticMapToOdom();
    }
  }

  // PX4 的 FRD->NED 四元數換算成 ENU 的 yaw（和發 TF 時用同一條公式）
  double currentEnuYaw() const
  {
    const tf2::Quaternion q_frd_to_ned(
      attitude_.q[1], attitude_.q[2], attitude_.q[3], attitude_.q[0]);
    const tf2::Quaternion q = qEnuToNed().inverse() * q_frd_to_ned * qFluToFrd();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

  static double wrapPi(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  // ---------------------------------------------------------------------------
  //  核心：把 PX4 的 NED / FRD 翻譯成 ROS 的 ENU / FLU
  // ---------------------------------------------------------------------------
  void publishOdomToBase()
  {
    if (!has_position_ || !has_attitude_) {return;}

    // 位置估計無效時不要發 TF。發了的話下游會拿到一個看似合理但其實是垃圾的
    // 位置，而且完全沒有跡象 —— 寧可讓 TF 斷掉，那樣 Nav2 會明確抱怨。
    if (!local_position_.xy_valid || !local_position_.z_valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "位置估計無效（xy_valid=%d z_valid=%d），暫停發布 TF",
        local_position_.xy_valid, local_position_.z_valid);
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;

    // --- 位置：NED -> ENU ---
    // GZBridge.cpp:600-602 做的是反方向（ENU->NED）：
    //     N = y_enu,  E = x_enu,  D = -z_enu
    // 這個對應是自己的反函數，所以反過來寫就是：
    //     x_enu(東) = E,  y_enu(北) = N,  z_enu(上) = -D
    t.transform.translation.x = local_position_.y;    // 東 <- PX4 的 y(東)
    t.transform.translation.y = local_position_.x;    // 北 <- PX4 的 x(北)
    t.transform.translation.z = -local_position_.z;   // 上 <- PX4 的 -z(下)

    // --- 姿態：FRD->NED 轉成 FLU->ENU ---
    // GZBridge.cpp:949 的正向公式是
    //     q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse()
    // 左右各乘反元素移項，得到我們要的反方向：
    //     q_FLU_to_ENU = q_ENU_to_NED.Inverse() * q_FRD_to_NED * q_FLU_to_FRD
    //
    // ⚠️ PX4 的 q[] 是 (w,x,y,z)，tf2::Quaternion 是 (x,y,z,w)，順序要換。
    const tf2::Quaternion q_frd_to_ned(
      attitude_.q[1], attitude_.q[2], attitude_.q[3], attitude_.q[0]);
    const tf2::Quaternion q_flu_to_enu =
      qEnuToNed().inverse() * q_frd_to_ned * qFluToFrd();

    t.transform.rotation.x = q_flu_to_enu.x();
    t.transform.rotation.y = q_flu_to_enu.y();
    t.transform.rotation.z = q_flu_to_enu.z();
    t.transform.rotation.w = q_flu_to_enu.w();

    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry od;
    od.header = t.header;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = t.transform.translation.x;
    od.pose.pose.position.y = t.transform.translation.y;
    od.pose.pose.position.z = t.transform.translation.z;
    od.pose.pose.orientation = t.transform.rotation;
    // 速度也一起轉成 ENU（PX4 給的是 NED）。
    // 注意這是「世界座標系的速度」，而 Odometry 的慣例是 child_frame（機體）——
    // Nav2 目前只用到位置，所以先這樣；真的要用速度時記得轉。
    od.twist.twist.linear.x = local_position_.vy;
    od.twist.twist.linear.y = local_position_.vx;
    od.twist.twist.linear.z = -local_position_.vz;
    odom_pub_->publish(od);

    if (!first_tf_logged_) {
      first_tf_logged_ = true;
      RCLCPP_INFO(get_logger(),
        "已開始發布 %s -> %s（首筆 ENU：東 %+.2f 北 %+.2f 上 %+.2f）",
        odom_frame_.c_str(), base_frame_.c_str(),
        t.transform.translation.x, t.transform.translation.y,
        t.transform.translation.z);
    }
  }

  std::string px4_namespace_, map_frame_, odom_frame_, base_frame_;
  bool publish_map_to_odom_{true};
  std::array<double, 3> odom_origin_{{0.0, 0.0, 0.0}};
  double odom_origin_yaw_{0.0};
  bool align_yaw_on_start_{false}, static_sent_{false};
  double initial_map_yaw_{0.0};
  // 等 EKF 航向收斂的秒數。太短會取到還在跳的值，太長沒必要。
  double align_delay_s_{5.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool has_position_{false}, has_attitude_{false}, first_tf_logged_{false};

  px4_msgs::msg::VehicleLocalPosition local_position_;
  px4_msgs::msg::VehicleAttitude attitude_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TfNode>());
  rclcpp::shutdown();
  return 0;
}
