#!/usr/bin/env python3
# =============================================================================
#  t_planner_check.py — 全域路徑規劃驗證（S5）
#
#  用法：
#      python3 src/drone_control/test/t_planner_check.py
#      python3 src/drone_control/test/t_planner_check.py --gui
#
#  回傳值：全部通過 0，有任何一項失敗 1。
#
#  ⚠️ 會開 Gazebo，但「不會起飛」——規劃器只算路徑，不碰飛機。
#
#  為什麼需要這支：
#      規劃器算出來的路徑「看起來像一條線」，對不對很難用眼睛judge ——
#      尤其是「有沒有貼著牆」「有沒有從 5 公分的縫鑽過去」這種。
#      而路徑錯了不會報錯，要等 S6 讓飛機跟著飛才會撞上去。
#
#  怎麼驗：
#      直接呼叫 planner_server 的 /compute_path_to_pose action，
#      拿回來的路徑逐點跟 .sdf 的牆比對。
#      同時驗「該失敗的時候要失敗」——目標放在牆裡面，規劃器應該乾脆地回報失敗，
#      而不是畫一條穿牆的線。
#
#  它抓不到什麼：
#      - 控制器追不追得上這條路徑（S6）
#      - 路徑好不好飛（轉彎會不會太急）——這一層只驗「安不安全」
# =============================================================================

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 共用 S4 那支的環境啟動與場景解析，不要複製一份 ——
# 複製的話兩邊會各自演化，之後改了場地只改到一邊。
from t_costmap_check import (            # noqa: E402
    Report, Procs, build_env, locate_arena, parse_walls, point_in_wall,
    NAMESPACE, INSTANCE, MODEL, WORLD, SPAWN, SPAWN_X, SPAWN_Y,
)

# 路徑上任何一點離牆至少要有這麼遠（公尺）。
# 依據：nav2_params.yaml 的 robot_radius 0.5，那是「絕對禁區」。
# 留 0.05 給格點取樣的誤差。
MIN_CLEARANCE = 0.45


def wall_clearance(px, py, walls):
    """點到最近一面牆的距離。在牆裡面回傳 0。"""
    best = 1e9
    for _, cx, cy, sx, sy, yaw in walls:
        a = math.radians(yaw)
        dx, dy = px - cx, py - cy
        lx = dx * math.cos(a) + dy * math.sin(a)
        ly = -dx * math.sin(a) + dy * math.cos(a)
        ox = max(abs(lx) - sx / 2.0, 0.0)
        oy = max(abs(ly) - sy / 2.0, 0.0)
        best = min(best, math.hypot(ox, oy))
    return best


class Planner:
    def __init__(self, node):
        import rclpy
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        from geometry_msgs.msg import PoseStamped
        self.node = node
        self.PoseStamped = PoseStamped
        self.Action = ComputePathToPose
        self.client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        self.last_frame = ""
        # 訂 costmap，出問題時可以查「規劃器眼中那一格是什麼」
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        self.grid = None
        node.create_subscription(
            OccupancyGrid, "/global_costmap/costmap",
            lambda m: setattr(self, "grid", m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def cost_at(self, wx, wy):
        g = self.grid
        if g is None:
            return None
        col = int((wx - g.info.origin.position.x) / g.info.resolution)
        row = int((wy - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= col < g.info.width and 0 <= row < g.info.height):
            return None
        return g.data[row * g.info.width + col]

    def ready(self, timeout=60.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def plan_retry(self, sx, sy, gx, gy, tries=5, timeout=20.0):
        """重試幾次再放棄。

        planner_server 的 action server 起來之後，costmap 還要一小段時間才
        真正可用；這期間送 goal 會被直接拒絕。等待 action server 不足以判斷
        「可以規劃了」—— 這是實測踩到的。
        """
        last = "沒試過"
        for i in range(tries):
            pts, err = self.plan(sx, sy, gx, gy, timeout)
            if pts is not None or (err and "拒絕" not in err):
                return pts, err
            last = err
            time.sleep(2.0)
        return None, f"{last}（重試 {tries} 次）"

    def plan(self, sx, sy, gx, gy, timeout=20.0):
        """回傳 (路徑點列表, 錯誤訊息)。規劃失敗時路徑是 None。"""
        import rclpy
        g = self.Action.Goal()
        g.use_start = True          # 明確指定起點，不用飛機當下的位置
        for pose, (x, y) in ((g.start, (sx, sy)), (g.goal, (gx, gy))):
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
        g.planner_id = "GridBased"

        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, "goal 被拒絕"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if not rf.done():
            return None, "等結果逾時"
        res = rf.result()
        from rclpy.action.client import GoalStatus
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            return None, f"規劃失敗（status={res.status}）"
        self.last_frame = res.result.path.header.frame_id
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        return pts, None


def report_path(rep, planner, pts, walls, label):
    """印出路徑的健檢結果，並把離牆最近的幾個點連同 costmap 成本一起列出來。

    列出 costmap 成本是關鍵：如果那一格顯示 0（自由），代表規劃器眼中
    那裡真的沒有障礙物 —— 問題在 costmap；如果顯示 100 卻還是規劃過去，
    問題就在規劃器的設定。兩者的修法完全不同。
    """
    length = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
                 for i in range(1, len(pts)))
    print(f"     {label}：{len(pts)} 點，長度 {length:.1f} m，"
          f"frame={planner.last_frame}")
    print(f"     頭尾：({pts[0][0]:.2f},{pts[0][1]:.2f}) -> "
          f"({pts[-1][0]:.2f},{pts[-1][1]:.2f})")
    scored = sorted(((wall_clearance(px, py, walls), px, py) for px, py in pts),
                    key=lambda t: t[0])
    worst = scored[0][0]
    if worst < MIN_CLEARANCE:
        print("     離牆最近的幾個點（附 costmap 上的實際成本）：")
        for c, px, py in scored[:6]:
            v = planner.cost_at(px, py)
            print(f"       ({px:7.2f}, {py:7.2f})  離牆 {c:.2f} m  costmap={v}")
        print("       costmap：100=障礙物 99=禁區 1~98=膨脹 0=自由 -1=未知")
    return worst, (scored[0][1], scored[0][2]), length


def main():
    ap = argparse.ArgumentParser(description="全域路徑規劃驗證（開 Gazebo，不飛）")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    args = ap.parse_args()

    import subprocess, json
    rep = Report()
    procs = Procs()
    scratch = f"/tmp/tplanner_{os.getpid()}"
    os.makedirs(scratch, exist_ok=True)
    node = None

    try:
        rep.section("C0 環境")
        arena = locate_arena()
        if not arena:
            rep.fail("找不到 drone_nav2_apriltag")
            return 1
        walls = parse_walls(os.path.join(arena, "gz", "worlds", f"{WORLD}.sdf"))
        rep.ok(f"world 裡有 {len(walls)} 個方塊障礙物")

        nodes = {}
        gj = os.path.join(arena, "graphs", "nav2_arena.geojson")
        for f in json.load(open(gj))["features"]:
            p = f.get("properties", {})
            if "id" in p and f["geometry"]["type"] == "Point":
                nodes[p["id"]] = tuple(f["geometry"]["coordinates"])
        rep.ok(f"拓樸圖有 {len(nodes)} 個節點")

        build = os.path.join(args.px4_dir, "build", "px4_sitl_default")
        for c in ("px4", "ruby", "planner_server", "map_server",
                  "lifecycle_manager", "px4_tf_node"):
            subprocess.run(["pkill", "-x", c], capture_output=True)
        time.sleep(2)
        env = build_env(args.px4_dir, arena, headless=not args.gui)

        rep.section("C1 啟動（規劃器只算路徑，不碰飛機）")
        if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"],
                          capture_output=True).returncode != 0:
            procs.start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888"],
                        env=env, log=f"{scratch}/agent.log")
            time.sleep(2)

        work = os.path.join(build, f"instance_{INSTANCE}")
        os.makedirs(work, exist_ok=True)
        penv = dict(env)
        penv.update({"PX4_UXRCE_DDS_NS": NAMESPACE, "PX4_SYS_AUTOSTART": "4001",
                     "PX4_SIM_MODEL": MODEL, "PX4_GZ_MODEL_POSE": SPAWN})
        procs.start("px4", [os.path.join(build, "bin", "px4"), "-i",
                            str(INSTANCE), "-d", os.path.join(build, "etc")],
                    env=penv, log=f"{scratch}/px4.log", cwd=work)
        ok = False
        for _ in range(90):
            time.sleep(1)
            if f"/world/{WORLD}/clock" in subprocess.run(
                    ["gz", "topic", "-l"], capture_output=True,
                    text=True, env=env).stdout:
                ok = True
                break
        if not ok:
            rep.fail("Gazebo 90 秒內沒起來")
            return 1
        rep.ok("Gazebo 已啟動")
        time.sleep(8)

        procs.start("bridge", ["ros2", "launch", "drone_control",
                               "px4_bridge.launch.py", f"namespace:={NAMESPACE}",
                               f"odom_origin:={SPAWN_X},{SPAWN_Y},0"],
                    env=env, log=f"{scratch}/bridge.log")
        procs.start("sensors", ["ros2", "launch", "drone_nav2_apriltag",
                                "cameras.launch.py", f"namespace:={NAMESPACE}",
                                "view:=false", "lidar:=true"],
                    env=env, log=f"{scratch}/sensors.log")
        time.sleep(5)
        procs.start("nav2", ["ros2", "launch", "drone_control",
                             "nav2.launch.py", f"namespace:={NAMESPACE}",
                             "goal_tool:=false"],
                    env=env, log=f"{scratch}/nav2.log")
        rep.ok("px4_bridge / 感測器 / costmap + planner 都已啟動")

        import rclpy
        rclpy.init()
        node = rclpy.create_node(
            "t_planner_check",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        planner = Planner(node)
        if not planner.ready(timeout=90.0):
            rep.fail("等不到 /compute_path_to_pose —— "
                     "planner_server 沒 activate，看 nav2.log")
            return 1
        rep.ok("planner_server 的 action server 已就緒")
        # action server 起來 != costmap 可用。多等一下，並讓 costmap 收到第一筆光達。
        for _ in range(60):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(5)

        # ---------------- C2 全程規劃 ----------------
        rep.section("C2 從起點規劃到降落點")
        s = nodes[min(nodes)]
        g = nodes[max(nodes)]
        pts, err = planner.plan_retry(s[0], s[1], g[0], g[1])
        if pts is None:
            rep.fail(f"規劃失敗：{err}。這條路應該走得通（拓樸圖驗過連通）")
        else:
            worst, worst_at, length = report_path(
                rep, planner, pts, walls,
                f"節點 {min(nodes)} -> 節點 {max(nodes)}")
            direct = math.hypot(g[0] - s[0], g[1] - s[1])
            print(f"     直線距離 {direct:.1f} m，繞路比 {length / direct:.2f}")
            if worst < MIN_CLEARANCE:
                rep.fail(f"路徑最近處離牆只有 {worst:.2f} m"
                         f"（在 ({worst_at[0]:.1f}, {worst_at[1]:.1f})，"
                         f"要 ≥ {MIN_CLEARANCE}）—— 飛過去會刮到")
            else:
                rep.ok(f"規劃成功且全程離牆 ≥ {worst:.2f} m"
                       f"（{len(pts)} 點，{length:.1f} m）")

            # 繞路比太誇張代表 costmap 有問題（把某條路封死了）
            if length / direct > 2.0:
                rep.warn(f"繞路比 {length / direct:.2f} 偏高，"
                         "可能是 inflation_radius 太大把通道封住了")

        # ---------------- C3 每一段相鄰節點 ----------------
        rep.section("C3 拓樸圖的每一段都規劃得出來")
        import itertools
        links = []
        for f in json.load(open(gj))["features"]:
            p = f.get("properties", {})
            if f["geometry"]["type"] == "LineString" and "id" in p:
                pass
        # geojson 的邊用座標表示，直接用相鄰節點對代替
        pairs = [(a, b) for a, b in itertools.combinations(sorted(nodes), 2)
                 if math.hypot(nodes[a][0] - nodes[b][0],
                               nodes[a][1] - nodes[b][1]) < 8.0]
        bad = []
        for a, b in pairs:
            pts2, err2 = planner.plan_retry(*nodes[a], *nodes[b], tries=2)
            if pts2 is None:
                bad.append(f"{a}->{b}({err2})")
            else:
                w = min(wall_clearance(px, py, walls) for px, py in pts2)
                if w < MIN_CLEARANCE:
                    bad.append(f"{a}->{b}(離牆 {w:.2f} m)")
        if bad:
            rep.fail(f"這些段有問題：{'、'.join(bad[:5])}")
        else:
            rep.ok(f"{len(pairs)} 段相鄰節點全部規劃成功且離牆足夠")

        # ---------------- C4 該失敗的要失敗 ----------------
        rep.section("C4 目標放在牆裡面，應該乾脆地失敗")
        wname, wx, wy = walls[0][0], walls[0][1], walls[0][2]
        pts3, err3 = planner.plan_retry(s[0], s[1], wx, wy, tries=2)
        if pts3 is not None:
            w, _, _ = report_path(rep, planner, pts3, walls,
                                  f"目標設在 {wname} 正中央")
            if w < MIN_CLEARANCE:
                rep.fail(f"目標放在牆 {wname} 的正中央 ({wx:.1f},{wy:.1f})，"
                         f"規劃器卻畫了一條進去的路（離牆 {w:.2f} m）")
            else:
                rep.ok(f"目標在牆裡，規劃器停在 {w:.2f} m 外（tolerance 生效）")
        else:
            rep.ok(f"目標在牆 {wname} 裡面，規劃器正確地回報失敗")

        rep.section("C5 目標放在地圖外，應該失敗")
        pts4, err4 = planner.plan(s[0], s[1], 999.0, 999.0)
        if pts4 is None:
            rep.ok("超出地圖範圍的目標被正確拒絕")
        else:
            rep.fail("目標在地圖外卻規劃成功了 —— allow_unknown 可能設成 true")

    except KeyboardInterrupt:
        print("\n(中斷)")
        return 130
    finally:
        if node is not None:
            try:
                node.destroy_node()
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
        procs.stop_all()
        print(f"   log：{scratch}")

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過" + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
