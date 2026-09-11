// =============================================================================
//  goal_to_planner_node —— 在 RViz 點一下就規劃路徑（S5 的小工具）
//
//  它做什麼：
//      訂 /goal_pose              RViz 的「2D Goal Pose」按鈕發的
//      呼叫 /compute_path_to_pose  planner_server 的 action
//      發 /plan                    規劃出來的路徑，給 RViz 畫
//
//  為什麼需要它：
//      RViz 的「2D Goal Pose」發的是 /goal_pose，而那個 topic 平常是由
//      bt_navigator 接收的 —— 但 bt_navigator 要 S6 才會啟動。
//      在只有 planner 的階段（S5），點下去不會有任何反應。
//      這支就是中間那一小段翻譯：把「點一下」變成「呼叫規劃器」。
//
//      S6 之後 bt_navigator 上線，這支就不需要了（同時開會重複規劃），
//      到時候把 launch 的 goal_tool 參數關掉即可。
//
//  刻意「只規劃、不執行」：
//      這一步要驗的是「路徑畫得對不對」，讓飛機動是 S6 的事。
//      規劃器算完就結束，不會有任何東西碰到 trajectory_setpoint。
// =============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>

using namespace std::chrono_literals;

class GoalToPlannerNode : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

  GoalToPlannerNode()
  : rclcpp::Node("goal_to_planner_node")
  {
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    action_name_ = declare_parameter<std::string>(
      "action_name", "compute_path_to_pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "plan");

    // RViz 的「2D Goal Pose」用預設 QoS（reliable / volatile / depth 1）
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 1,
      std::bind(&GoalToPlannerNode::onGoal, this, std::placeholders::_1));

    // 路徑用 transient_local：RViz 晚一點開也看得到最後一條路徑，
    // 不然每次重開 RViz 都要重點一次目標。
    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

    client_ = rclcpp_action::create_client<ComputePath>(this, action_name_);

    RCLCPP_INFO(get_logger(), "===== goal_to_planner_node =====");
    RCLCPP_INFO(get_logger(), "  在 RViz 按「2D Goal Pose」點一個位置就會規劃");
    RCLCPP_INFO(get_logger(), "  訂閱 : %s", goal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  呼叫 : %s（planner_id=%s）",
      action_name_.c_str(), planner_id_.c_str());
    RCLCPP_INFO(get_logger(), "  發布 : %s", path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  ⚠️ 只規劃，不會叫飛機動（那是 S6）");
  }

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(),
        "等不到 %s —— planner_server 沒開，或還沒 activate（看 lifecycle_manager）",
        action_name_.c_str());
      return;
    }

    ComputePath::Goal goal;
    goal.goal = *msg;
    goal.planner_id = planner_id_;
    // use_start = false 代表「從飛機現在的位置開始規劃」。
    // 設 true 的話要自己填 start，那是給離線規劃用的。
    goal.use_start = false;

    RCLCPP_INFO(get_logger(), "收到目標 (%.2f, %.2f)，開始規劃…",
      msg->pose.position.x, msg->pose.position.y);

    rclcpp_action::Client<ComputePath>::SendGoalOptions opts;
    opts.result_callback = [this](const GoalHandle::WrappedResult & r) {
      if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
        // 目標卡在牆裡、或被 costmap 的膨脹層包住時會走到這裡。
        // 這是「正確的失敗」—— 規劃器不該畫一條穿牆的路出來。
        RCLCPP_WARN(get_logger(),
          "規劃失敗（code=%d）。常見原因：目標在障礙物裡、"
          "或起點/終點超出地圖範圍", static_cast<int>(r.code));
        return;
      }
      const auto & path = r.result->path;
      if (path.poses.empty()) {
        RCLCPP_WARN(get_logger(), "規劃成功但路徑是空的");
        return;
      }
      double len = 0.0;
      for (size_t i = 1; i < path.poses.size(); ++i) {
        len += std::hypot(
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
      }
      const double t = r.result->planning_time.sec +
        r.result->planning_time.nanosec * 1e-9;
      RCLCPP_INFO(get_logger(),
        "規劃成功：%zu 個點，長度 %.2f m，耗時 %.0f ms（frame=%s）",
        path.poses.size(), len, t * 1000.0, path.header.frame_id.c_str());
      path_pub_->publish(path);
    };
    client_->async_send_goal(goal, opts);
  }

  std::string planner_id_, action_name_, goal_topic_, path_topic_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Client<ComputePath>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
