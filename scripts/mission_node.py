#!/usr/bin/env python3
# =============================================================================
#  mission_node.py — S7：整趟任務的編排者
#
#  用法：
#      ros2 launch drone_control mission.launch.py
#      ros2 launch drone_control mission.launch.py goal_node:=7
#      ros2 launch drone_control mission.launch.py land:=false
#
#  四個階段：
#      ROUTE     問 route_server「該走哪些節點」
#      TAKEOFF   解鎖 + 起飛到定高（借 arm_and_takeoff.py 的類別，不重寫）
#      NAVIGATE  逐個節點送 NavigateToPose —— 避障由 Nav2 負責
#      LAND      最後一個節點到了 → PrecisionLand 精準降落
#
#  為什麼需要這支（S6 之後還缺的三件事）：
#      1. 在這之前「飛到了」是猜的。nav2_then_land.launch.py 用
#         OnProcessExit 觸發降落，而那個事件的意思是「子程序結束了」，
#         不是「到目標了」—— fly_nodes 因為錯誤退出時，降落照樣會被觸發，
#         然後在錯的地方降。這支改成等 NavigateToPose 的 result。
#      2. S4–S6 做好的 Nav2 只有在 RViz 手點目標時才會動。fly_nodes 是
#         自己發 TrajectorySetpoint 走直線（見它自己的註解：「避障是階段
#         4、6 的事」），不會避障。這支才讓 Nav2 真的進到任務流程裡。
#      3. PrecisionLand 回的 result_code（NO_TAG / BAD_POSE / REJECTED）
#         在這之前沒有任何程式在讀。
#
#  前置條件（這支都不會幫你開）：
#      start_arena_sitl.sh / MicroXRCEAgent / px4_bridge.launch.py /
#      cameras.launch.py(lidar:=true) / nav2.launch.py(level:=full) /
#      route_server（mission.launch.py 會幫你開這個）
#
#  ⚠️ 為什麼先問路線、後起飛：
#      拓樸圖壞掉（節點 id 打錯、geojson 路徑不對）是最常見的失敗，
#      而那種失敗完全不需要飛機在空中就能發現。先問路線的話，
#      圖有問題時飛機根本不會解鎖。
# =============================================================================

import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, NavigateToPose

# 起飛邏輯「借用」不「複製」—— 複製一份的話兩邊會各自演化，
# 改了一邊另一邊悄悄過期。安裝後 arm_and_takeoff.py 和這支都在
# lib/drone_control/ 同一層，所以這個寫法在原始碼和安裝後都成立。
# （同樣的手法見 test/t_nav2_flight_check.py:36）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_and_takeoff import ArmAndTakeoff          # noqa: E402

S_ROUTE = "ROUTE"
S_TAKEOFF = "TAKEOFF"
S_NAVIGATE = "NAVIGATE"
S_LAND = "LAND"
S_DONE = "DONE"

# 離開碼：讓上層（launch、測試、CI）分得出是哪一段掛掉
RC_OK = 0
RC_ROUTE = 1
RC_TAKEOFF = 2
RC_NAVIGATE = 3
RC_LAND = 4


class Mission(Node):
    def __init__(self):
        # ⚠️ use_sim_time 一定要 true。NavigateToPose 的 goal 帶時間戳，
        #    而 Nav2 整套跑在 Gazebo 的 /clock 上。用系統時間的話兩者差
        #    約 1.79e9 秒，Nav2 會查不到那個時刻的 TF，goal 直接失敗。
        super().__init__(
            "mission_node",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.ns = p("namespace", "MAV1")
        self.graph_file = p("graph_file", "")
        self.start_node = p("start_node", -1)
        self.goal_node = p("goal_node", -1)
        self.altitude = p("flight_altitude", 3.0)
        self.target_system = p("target_system", 1)
        self.do_land = p("land", True)
        # 每段導航的上限。一段最長是 0→1 的 5.4 m，巡航約 1 m/s，
        # 加上繞障與復原行為的餘裕，120 秒很寬鬆。
        self.nav_timeout = p("nav_timeout", 120.0)
        self.land_timeout = p("land_timeout", 180.0)
        self.approach_altitude = p("approach_altitude", 0.0)
        self.align_yaw = p("align_yaw", True)

        self.state = S_ROUTE
        self.route = []                    # [(id, x, y), ...]

        self.route_client = ActionClient(self, ComputeRoute, "compute_route")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # PrecisionLand 的型別是延後 import 的，見 land()

        self._last_nav_log = 0.0
        self._last_land_state = None

    # -------------------------------------------------------------------------
    #  小工具
    # -------------------------------------------------------------------------

    def _goto(self, state, why=""):
        self.get_logger().info(
            f"[狀態] {self.state} -> {state}" + (f"  （{why}）" if why else ""))
        self.state = state

    def _sleep(self, seconds):
        """等待但繼續 spin —— 單純 time.sleep() 會讓 action client 的
        回呼堆在佇列裡不處理。"""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait(self, future, timeout, what):
        """自己寫等待迴圈，不用 spin_until_future_complete。

        原因：那支的 timeout 行為跟 use_sim_time 綁在一起不好預測，
        而這裡要的是「牆上時鐘幾秒」——（模擬卡住時也要能逾時退出）。
        """
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f"{what} 逾時（{timeout:.0f} 秒）")
            return None
        return future.result()

    def _send_and_wait(self, client, goal, what, timeout,
                       feedback_cb=None, server_timeout=20.0):
        """送 goal、等被接受、等 result。回傳 (status, result) 或 None。"""
        if not client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error(
                f"等不到 {what} 的 action server —— 它有開嗎？")
            return None

        sent = client.send_goal_async(goal, feedback_callback=feedback_cb)
        handle = self._wait(sent, server_timeout, f"{what} 送出 goal")
        if handle is None:
            return None
        if not handle.accepted:
            self.get_logger().error(f"{what} 拒絕了這個 goal")
            return None

        res = self._wait(handle.get_result_async(), timeout, what)
        if res is None:
            # 逾時就主動取消，不然 server 會一直跑下去（飛機會繼續飛）
            handle.cancel_goal_async()
            return None
        return (res.status, res.result)

    # -------------------------------------------------------------------------
    #  ROUTE
    # -------------------------------------------------------------------------

    def _resolve_endpoints(self):
        """把 -1 換成實際的節點 id。

        只為了這件事讀 geojson —— 節點「座標」是從 route_server 的回覆拿的，
        不是從這裡讀。理由：座標要跟 route_server 規劃時用的那一份一致，
        各讀一次就有「兩份來源」的風險。
        """
        if self.start_node >= 0 and self.goal_node >= 0:
            return True
        if not self.graph_file or not os.path.isfile(self.graph_file):
            self.get_logger().error(
                f"start_node/goal_node 有 -1，需要讀拓樸圖來決定，"
                f"但讀不到：{self.graph_file or '(未設定)'}")
            return False
        try:
            with open(self.graph_file, encoding="utf-8") as f:
                data = json.load(f)
            ids = [feat["properties"]["id"] for feat in data["features"]
                   if feat["geometry"]["type"] == "Point"]
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().error(f"拓樸圖解析失敗：{e}")
            return False
        if not ids:
            self.get_logger().error("拓樸圖裡沒有任何節點")
            return False
        if self.start_node < 0:
            self.start_node = min(ids)
        if self.goal_node < 0:
            self.goal_node = max(ids)
        return True

    def plan_route(self):
        if not self._resolve_endpoints():
            return False

        goal = ComputeRoute.Goal()
        goal.start_id = int(self.start_node)
        goal.goal_id = int(self.goal_node)
        # 用節點 id 查詢就不會去碰 TF。這裡本來就知道要從哪個節點出發，
        # 沒必要讓 route_server 再去猜一次。
        goal.use_start = False
        goal.use_poses = False

        self.get_logger().info(
            f"向 route_server 要路線：節點 {self.start_node} -> {self.goal_node}")
        out = self._send_and_wait(self.route_client, goal, "compute_route", 30.0)
        if out is None:
            return False
        status, result = out
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"route_server 規劃失敗（status={status}）")
            return False

        self.route = [(n.nodeid, n.position.x, n.position.y)
                      for n in result.route.nodes]
        if not self.route:
            self.get_logger().error("route_server 回了空路線")
            return False

        ids = " -> ".join(str(n[0]) for n in self.route)
        self.get_logger().info(
            f"拿到路線（cost {result.route.route_cost:.2f}）：{ids}")
        return True

    # -------------------------------------------------------------------------
    #  TAKEOFF
    # -------------------------------------------------------------------------

    def takeoff(self):
        self.get_logger().info(f"起飛到 {self.altitude:.1f} m")
        # 同一個 process 裡多開一個 Node 是允許的。ArmAndTakeoff.run()
        # 自己跑 spin_once 迴圈，期間 mission_node 沒有待辦工作，不會打到。
        node = ArmAndTakeoff(self.ns, self.altitude, self.target_system)
        try:
            rc = node.run()
        finally:
            node.destroy_node()
        return rc == 0

    # -------------------------------------------------------------------------
    #  NAVIGATE
    # -------------------------------------------------------------------------

    def _nav_feedback(self, msg):
        # 每 2 秒印一次就好，不然 10 Hz 的回饋會把畫面沖掉
        now = time.monotonic()
        if now - self._last_nav_log < 2.0:
            return
        self._last_nav_log = now
        fb = msg.feedback
        extra = ""
        if fb.number_of_recoveries > 0:
            # 復原行為被觸發代表「卡住過」—— 這是之後調參數最有用的線索
            extra = f"，已觸發 {fb.number_of_recoveries} 次復原行為"
        self.get_logger().info(
            f"    剩 {fb.distance_remaining:.2f} m{extra}")

    def navigate(self):
        total = len(self.route)
        for i, (nid, x, y) in enumerate(self.route, start=1):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            # z 一律 0：Nav2 是 2D 的，高度由 cmd_vel_to_px4_node 自己維持。
            pose.pose.orientation.w = 1.0      # 朝向不管，見下方註解

            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(
                f"  [{i}/{total}] 前往節點 {nid} ({x:+.2f}, {y:+.2f})")
            self._last_nav_log = 0.0
            out = self._send_and_wait(
                self.nav_client, goal, f"navigate_to_pose（節點 {nid}）",
                self.nav_timeout, feedback_cb=self._nav_feedback)
            if out is None:
                return False
            status, _ = out
            # ⚠️ Humble 的 NavigateToPose.Result 是 std_msgs/Empty，
            #    裡面沒有任何錯誤碼 —— 成功與否只能看 goal 的 status。
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"  節點 {nid} 導航失敗（status={status}）")
                return False
            self.get_logger().info(f"  ✓ 到達節點 {nid}")
        return True

    # -------------------------------------------------------------------------
    #  LAND
    # -------------------------------------------------------------------------

    def _land_feedback(self, msg):
        # 只在狀態「改變」時印 —— 精準降落的回饋是高頻的，
        # 每則都印會看不出轉換點在哪裡。
        fb = msg.feedback
        if fb.state == self._last_land_state:
            return
        self._last_land_state = fb.state
        names = {0: "SEARCHING", 1: "ALIGNING", 2: "DESCENDING", 3: "HANDOFF"}
        self.get_logger().info(
            f"    降落狀態 -> {names.get(fb.state, fb.state)}"
            f"（高度 {fb.altitude:.2f} m，水平誤差 {fb.xy_error:.2f} m，"
            f"看得到 tag：{'是' if fb.tag_visible else '否'}）")

    def land(self):
        # 延後 import：降落是選用的。沒裝 drone_apriltag_landing 的人
        # 照樣可以用這支跑「導航到某個節點」，只是不能降落 ——
        # 寫在檔頭 import 的話，沒裝就連啟動都啟動不了。
        try:
            from drone_apriltag_landing.action import PrecisionLand
        except ImportError:
            self.get_logger().error(
                "找不到 drone_apriltag_landing 的 PrecisionLand action。"
                "沒裝那個套件的話請加 land:=false")
            return False

        client = ActionClient(self, PrecisionLand, f"/{self.ns}/precision_land")
        goal = PrecisionLand.Goal()
        goal.tag_id = -1                   # -1 = 沿用降落包參數檔裡的 tag_id
        goal.approach_altitude = float(self.approach_altitude)
        goal.align_yaw = bool(self.align_yaw)

        self.get_logger().info("交給 AprilTag 精準降落")
        self._last_land_state = None
        out = self._send_and_wait(
            client, goal, "precision_land", self.land_timeout,
            feedback_cb=self._land_feedback)
        if out is None:
            return False
        status, result = out
        codes = {0: "SUCCESS", 1: "TIMEOUT", 2: "CANCELLED",
                 3: "NO_TAG", 4: "BAD_POSE", 5: "REJECTED"}
        name = codes.get(result.result_code, result.result_code)
        if status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self.get_logger().error(f"降落失敗：{name}（status={status}）")
            return False
        self.get_logger().info(
            f"✓ 降落完成（{name}）　殘餘誤差 "
            f"x {result.final_x_error:+.3f} m、y {result.final_y_error:+.3f} m、"
            f"yaw {math.degrees(result.final_yaw_error):+.1f}°")
        return True

    # -------------------------------------------------------------------------
    #  主流程
    # -------------------------------------------------------------------------

    def run(self):
        self.get_logger().info("=" * 62)
        self.get_logger().info(f"任務開始　namespace={self.ns}　"
                               f"高度={self.altitude:.1f} m　"
                               f"降落={'要' if self.do_land else '不要'}")
        self.get_logger().info("=" * 62)

        if not self.plan_route():
            return RC_ROUTE

        self._goto(S_TAKEOFF)
        if not self.takeoff():
            return RC_TAKEOFF

        self._goto(S_NAVIGATE)
        if not self.navigate():
            return RC_NAVIGATE

        if not self.do_land:
            self._goto(S_DONE, "land:=false，停在終點上空")
            self.get_logger().info(
                "控制權交還：這支不再送 goal，PX4 會掉進 HOLD 原地懸停。")
            return RC_OK

        self._goto(S_LAND)
        # ⚠️ 不能馬上送降落 goal。precision_land_node.cpp:380 會檢查
        #    /{ns}/fmu/in/trajectory_setpoint 上的「發布者數量」，大於 1
        #    就拒絕接手（兩組 setpoint 交錯會讓飛機抽搐）。
        #    而 cmd_vel_to_px4_node 是等 cmd_timeout_s（預設 0.5 秒）沒收到
        #    cmd_vel 才 setpoint_pub_.reset() 把發布者銷毀（同檔 :140）。
        #    導航剛結束的那一刻發布者還在，直接送會拿到 CODE_REJECTED。
        self.get_logger().info("等 cmd_vel 的發布者收手（2 秒）")
        self._sleep(2.0)
        if not self.land():
            return RC_LAND

        self._goto(S_DONE, "全程完成")
        return RC_OK


def main():
    rclpy.init()
    node = Mission()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("(中斷)")
        rc = 130
    finally:
        node.destroy_node()
        # rclpy.shutdown() 不能放在 timer callback 裡（spin 會卡住不返回），
        # 但這裡是在 main 的同步流程中，安全。
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
