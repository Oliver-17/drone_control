#!/usr/bin/env python3
# =============================================================================
#  t_nav2_flight_check.py — Nav2 導航飛行驗證（S6）
#
#  用法：
#      python3 src/drone_control/test/t_nav2_flight_check.py
#      python3 src/drone_control/test/t_nav2_flight_check.py --gui
#
#  回傳值：全部通過 0，有任何一項失敗 1。
#
#  ⚠️ 這支會讓飛機在「有 15 m 高牆的場地裡」真的飛。
#     跟前幾支不同的是，這次有 costmap 和規劃器在避障 ——
#     這支要驗的就是「那個避障真的有用」。
#     速度上限設得保守（0.8 m/s），目標也挑拓樸圖上已知安全的節點。
#
#  為什麼需要這支：
#      前面每一層都驗過了，但「規劃得對」不等於「飛得到」。
#      控制器追不上路徑、轉彎切太內側、高度掉下去 —— 這些只有真的飛才知道。
#
#      而且飛行中會暴露一類前面看不到的問題：控制器輸出的 cmd_vel
#      經過 cmd_vel_to_px4_node 轉換後，方向對不對。
#      那條轉換鏈在 S2 用手動指令驗過，但沒有和 Nav2 串起來驗過。
#
#  它抓不到什麼：
#      - 真實的風擾與地效
#      - 三機編隊時的互相干擾
# =============================================================================

import argparse
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from t_costmap_check import (            # noqa: E402
    Report, Procs, build_env, set_sitl_params, locate_arena, parse_walls,
    NAMESPACE, INSTANCE, MODEL, WORLD, SPAWN, SPAWN_X, SPAWN_Y,
)
from t_planner_check import wall_clearance   # noqa: E402

FLIGHT_ALT = 3.0
# 飛行途中離牆的最低容許值。比規劃時的 0.45 再放寬一點 ——
# 控制器追路徑一定有誤差，要求它和規劃結果一樣精準不合理。
MIN_FLIGHT_CLEARANCE = 0.35
# 高度漂移容許值。定高是 cmd_vel_to_px4_node 用 P 控制器維持的。
MAX_ALT_DRIFT = 0.6


def gazebo_truth(env):
    """從 Gazebo 問「飛機真正在哪」。

    這是唯一能分辨「TF 算錯」和「光達量錯」的方法 ——
    前面所有的比對都是拿我們自己算的東西互相比，永遠自洽。
    """
    model = f"{MODEL}_{INSTANCE}"
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-n", "1",
             "-t", f"/world/{WORLD}/dynamic_pose/info"],
            capture_output=True, text=True, env=env, timeout=8).stdout
    except Exception:
        return None
    # 文字格式：一連串 pose { name: "..." position { x: .. y: .. } }
    blocks = out.split("pose {")
    for b in blocks:
        if f'name: "{model}"' not in b:
            continue
        try:
            pos = b.split("position {")[1].split("}")[0]
            vals = {}
            for line in pos.splitlines():
                line = line.strip()
                for k in ("x:", "y:", "z:"):
                    if line.startswith(k):
                        vals[k[0]] = float(line.split(":")[1])
            yaw_t = None
            if "orientation {" in b:
                o = b.split("orientation {")[1].split("}")[0]
                q = {}
                for line in o.splitlines():
                    line = line.strip()
                    for k in ("x:", "y:", "z:", "w:"):
                        if line.startswith(k):
                            q[k[0]] = float(line.split(":")[1])
                if len(q) == 4:
                    yaw_t = math.atan2(
                        2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                        1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
            if "x" in vals and "y" in vals:
                return vals["x"], vals["y"], vals.get("z", 0.0), yaw_t
        except (IndexError, ValueError):
            continue
    return None


class Flyer:
    def __init__(self, node, ns):
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from px4_msgs.msg import (BatteryStatus, FailsafeFlags, VehicleAttitude,
                                  VehicleLocalPosition, VehicleStatus)
        from sensor_msgs.msg import LaserScan
        import tf2_ros
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        self.node = node
        self.NavigateToPose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.pos = None
        self.track = []          # 全程軌跡，用來事後檢查有沒有貼牆
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(
            VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
            self._on_pos, qos)
        # 失效保護一旦觸發，飛機就不再聽 Nav2 了 ——
        # 光看 PX4 的 log 字串會誤判（"battery warning" 那個音效在別的情況也會響），
        # 訂旗標才知道真正的原因。
        self.failsafe = None
        self.battery = None
        self.nav_states = []
        node.create_subscription(FailsafeFlags, f"/{ns}/fmu/out/failsafe_flags",
                                 lambda m: setattr(self, "failsafe", m), qos)
        node.create_subscription(BatteryStatus, f"/{ns}/fmu/out/battery_status",
                                 lambda m: setattr(self, "battery", m), qos)
        node.create_subscription(
            VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
            lambda m: self.nav_states.append(m.nav_state)
            if not self.nav_states or self.nav_states[-1] != m.nav_state else None,
            qos)

        # ---- 翻覆診斷用 ----
        # 假設：機身傾斜時，朝下的光束會打到地板，而 min_obstacle_height=0.0
        # 讓那些點被當成障礙物 -> costmap 出現「空地上的假牆」-> 控制器急閃 ->
        # 傾角更大 -> 假牆更近 -> 正回饋 -> 翻機。
        # 這裡把「傾角」和「不落在真牆上的回波」同時記下來，才驗得出因果。
        self.tilts = []            # (時間, roll度, pitch度, 合成傾角度)
        node.create_subscription(
            VehicleAttitude, f"/{ns}/fmu/out/vehicle_attitude",
            self._on_att, qos)
        self.scan = None
        node.create_subscription(
            LaserScan, f"/{ns}/scan", lambda m: setattr(self, "scan", m), qos)
        self.buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buf, node)
        self.ghost_samples = []    # (傾角, 假回波比例, 最近的假回波距離)

    def _on_att(self, m):
        # PX4 的 q 是 (w,x,y,z)，FRD->NED
        w, x, y, z = m.q[0], m.q[1], m.q[2], m.q[3]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        self.tilts.append((time.time(), math.degrees(roll), math.degrees(pitch),
                           math.degrees(math.hypot(roll, pitch))))

    def gazebo_truth(self, env):
        # 實作抽到模組層了，S7 的 t_mission_check.py 要用同一份 ——
        # 複製一份的話，哪天 Gazebo 的輸出格式變了只會改到其中一邊。
        return gazebo_truth(env)

    def sample_ghosts(self, walls):
        """取樣一次：這一幀的光達回波有多少「不落在真牆上」，它們的高度多少。

        假回波的高度是關鍵證據：
          z 接近 0   -> 打到地板（傾斜造成的）
          z 接近飛行高度 -> 真的有東西，或 TF 算錯
        """
        import rclpy
        s_ = self.scan
        if s_ is None:
            return
        try:
            # ⚠️ 一定要用「這一幀掃描的時間戳」，不能用最新的 TF。
            #    飛行中機身在移動也在轉，100 ms 的時間差在 10 m 處
            #    就是 0.5 m 的誤差 —— 足以讓一大半的回波被誤判成「假的」。
            #    S4 已經踩過這個坑，這裡不能重蹈。
            tf = self.buf.lookup_transform(
                "map", s_.header.frame_id,
                rclpy.time.Time.from_msg(s_.header.stamp))
        except Exception:
            self.tf_fail = getattr(self, "tf_fail", 0) + 1
            return
        q = tf.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
              2 * (qx * qz + qy * qw)],
             [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
              2 * (qy * qz - qx * qw)],
             [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
              1 - 2 * (qx * qx + qy * qy)]]
        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oz = tf.transform.translation.z

        ghosts, total, nearest, zs = 0, 0, 1e9, []
        detail = []      # 留幾個假回波的實際座標，用來看它們到底是什麼
        # 每 4 點取一個就夠看趨勢，1080 點全算在 Python 裡太慢
        for i in range(0, len(s_.ranges), 4):
            r = s_.ranges[i]
            if not (0.6 < r < 12.0):      # 和 obstacle_max_range 一致
                continue
            a = s_.angle_min + i * s_.angle_increment
            vx, vy = math.cos(a), math.sin(a)
            px = ox + r * (R[0][0] * vx + R[0][1] * vy)
            py = oy + r * (R[1][0] * vx + R[1][1] * vy)
            pz = oz + r * (R[2][0] * vx + R[2][1] * vy)
            total += 1
            c = wall_clearance(px, py, walls)
            if c > 0.5:
                ghosts += 1
                nearest = min(nearest, r)
                zs.append(pz)
                if len(detail) < 8:
                    detail.append((px, py, pz, r, c))
        if total == 0:
            return
        tilt = self.tilts[-1][3] if self.tilts else 0.0
        # 假設：航向殘餘誤差 δ 會讓回波位置偏移 d·δ，d = 離 odom 原點的距離。
        # 如果「平均離牆距離 / 離原點距離」在整趟飛行中是個常數，
        # 那就證明是角度誤差，而不是光達雜訊或地板回波。
        d_origin = math.hypot(ox - SPAWN_X, oy - SPAWN_Y)
        mean_off = (sum(dd[4] for dd in detail) / len(detail)) if detail else 0.0
        self.offset_vs_dist = getattr(self, "offset_vs_dist", [])
        if detail and d_origin > 1.0:
            self.offset_vs_dist.append((d_origin, mean_off))
        self.ghost_samples.append(
            (tilt, ghosts / total * 100, nearest if ghosts else float("nan"),
             sum(zs) / len(zs) if zs else float("nan"), total,
             (ox, oy), detail))

    def _on_pos(self, m):
        self.pos = m
        # PX4 的 NED 是相對起飛點，換成 map（ENU、世界原點）才能跟牆比對
        self.track.append((SPAWN_X + m.y, SPAWN_Y + m.x, -m.z))

    def spin(self, seconds):
        import rclpy
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def navigate_sampling(self, gx, gy, walls, timeout=180.0):
        """送目標並「在飛行途中持續取樣」傾角與假回波。

        一定要飛行中取樣 —— 事後補抓沒有意義，翻覆前那幾秒的資料才是關鍵。
        """
        import rclpy
        if not self.client.wait_for_server(timeout_sec=30.0):
            return False, "等不到 navigate_to_pose action server"
        g = self.NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.w = 1.0

        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, "goal 被拒絕"
        rf = gh.get_result_async()
        end = time.time() + timeout
        nxt = 0.0
        while time.time() < end and not rf.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if time.time() >= nxt:
                self.sample_ghosts(walls)
                nxt = time.time() + 0.5      # 2 Hz 取樣
        if not rf.done():
            return False, f"導航逾時（{timeout:.0f} 秒）"
        from rclpy.action.client import GoalStatus
        st = rf.result().status
        if st != GoalStatus.STATUS_SUCCEEDED:
            return False, f"導航失敗（status={st}）"
        return True, "抵達"

    def navigate(self, gx, gy, timeout=180.0):
        """送一個 NavigateToPose 目標，等它跑完。回傳 (成功, 說明)。"""
        import rclpy
        if not self.client.wait_for_server(timeout_sec=30.0):
            return False, "等不到 navigate_to_pose action server"
        g = self.NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.w = 1.0

        fut = self.client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, "goal 被拒絕"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if not rf.done():
            return False, f"導航逾時（{timeout:.0f} 秒）"
        from rclpy.action.client import GoalStatus
        st = rf.result().status
        if st != GoalStatus.STATUS_SUCCEEDED:
            return False, f"導航失敗（status={st}）"
        return True, "抵達"


def main():
    ap = argparse.ArgumentParser(description="Nav2 導航飛行驗證（會真的飛）")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    args = ap.parse_args()

    import json
    rep = Report()
    procs = Procs()
    scratch = f"/tmp/tnav2_{os.getpid()}"
    os.makedirs(scratch, exist_ok=True)
    node = None

    try:
        rep.section("C0 環境")
        arena = locate_arena()
        if not arena:
            rep.fail("找不到 drone_nav2_apriltag")
            return 1
        walls = parse_walls(os.path.join(arena, "gz", "worlds", f"{WORLD}.sdf"))
        nodes = {}
        for f in json.load(open(os.path.join(
                arena, "graphs", "nav2_arena.geojson")))["features"]:
            p = f.get("properties", {})
            if "id" in p and f["geometry"]["type"] == "Point":
                nodes[p["id"]] = tuple(f["geometry"]["coordinates"])
        rep.ok(f"{len(walls)} 面牆 / {len(nodes)} 個拓樸節點")

        build = os.path.join(args.px4_dir, "build", "px4_sitl_default")
        for c in ("px4", "ruby", "planner_server", "controller_server",
                  "bt_navigator", "behavior_server", "map_server",
                  "lifecycle_manager", "px4_tf_node", "cmd_vel_to_px4"):
            subprocess.run(["pkill", "-x", c], capture_output=True)
        time.sleep(2)
        env = build_env(args.px4_dir, arena, headless=not args.gui)

        rep.section("C1 啟動完整的 Nav2 堆疊")
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

        if not set_sitl_params(build, env, INSTANCE, rep):
            return 1

        procs.start("bridge", ["ros2", "launch", "drone_control",
                               "px4_bridge.launch.py", f"namespace:={NAMESPACE}",
                               f"odom_origin:={SPAWN_X},{SPAWN_Y},0",
                               f"flight_altitude:={FLIGHT_ALT}"],
                    env=env, log=f"{scratch}/bridge.log")
        procs.start("sensors", ["ros2", "launch", "drone_nav2_apriltag",
                                "cameras.launch.py", f"namespace:={NAMESPACE}",
                                "view:=false", "lidar:=true"],
                    env=env, log=f"{scratch}/sensors.log")
        time.sleep(5)
        procs.start("nav2", ["ros2", "launch", "drone_control", "nav2.launch.py",
                             f"namespace:={NAMESPACE}", "level:=full"],
                    env=env, log=f"{scratch}/nav2.log")
        rep.ok("px4_bridge / 感測器 / Nav2（含 controller）都已啟動")

        import rclpy
        rclpy.init()
        node = rclpy.create_node(
            "t_nav2_flight_check",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        flyer = Flyer(node, NAMESPACE)

        # ---------------- C2 起飛 ----------------
        rep.section("C2 解鎖與起飛")
        # 借用 arm_and_takeoff.py，不要在測試裡複製一份起飛流程 ——
        # 複製的話兩邊會各自演化，改了一邊另一邊悄悄過期。
        r = subprocess.run(
            ["ros2", "run", "drone_control", "arm_and_takeoff.py",
             "--ns", NAMESPACE, "--altitude", str(FLIGHT_ALT)],
            env=env, capture_output=True, text=True, timeout=200)
        for line in r.stdout.strip().splitlines()[-4:]:
            print("     " + line)
        if r.returncode != 0:
            rep.fail("起飛失敗，看上面的訊息")
            return 1
        rep.ok(f"已懸停在 {FLIGHT_ALT} m")
        flyer.spin(3.0)
        flyer.track.clear()      # 起飛階段的軌跡不算，只看導航段

        # ---------------- C3 導航到鄰近節點 ----------------
        rep.section("C3 導航（Nav2 真的驅動飛機）")
        # 挑節點 1（0,5）—— 離起點 5.4 m，全程在起飛區內，最安全的第一趟
        # 用比較遠的目標才逼得出高速與大傾角。
        # 節點 1（5.4 m）太近，飛機還沒加速到會出事的程度就到了 ——
        # 這也是原本的測試沒抓到翻覆的原因。
        goal_id = 3
        gx, gy = nodes[goal_id]
        dist0 = math.hypot(gx - (SPAWN_X), gy - (SPAWN_Y))
        print(f"     目標：節點 {goal_id} ({gx:.1f}, {gy:.1f})，距離約 {dist0:.1f} m")
        t0 = time.time()
        good, why = flyer.navigate_sampling(gx, gy, walls, timeout=180.0)
        dt = time.time() - t0
        if not good:
            rep.fail(f"導航失敗：{why}。看 nav2.log 的 controller_server 訊息")
        else:
            fx, fy = SPAWN_X + flyer.pos.y, SPAWN_Y + flyer.pos.x
            err = math.hypot(fx - gx, fy - gy)
            print(f"     實際到達 ({fx:.2f}, {fy:.2f})，離目標 {err:.2f} m，"
                  f"耗時 {dt:.0f} 秒")
            if err > 1.0:
                rep.fail(f"停在離目標 {err:.2f} m 的地方（容許 1.0 m）")
            else:
                rep.ok(f"抵達目標，誤差 {err:.2f} m，耗時 {dt:.0f} 秒")

        # ---------------- C4 飛行品質 ----------------
        rep.section("C4 飛行品質（全程軌跡）")
        if len(flyer.track) < 20:
            rep.warn(f"只記錄到 {len(flyer.track)} 個位置點，樣本太少")
        else:
            alts = [z for _, _, z in flyer.track]
            drift = max(abs(a - FLIGHT_ALT) for a in alts)
            print(f"     高度：{min(alts):.2f} ~ {max(alts):.2f} m"
                  f"（目標 {FLIGHT_ALT}，最大偏差 {drift:.2f}）")
            if drift > MAX_ALT_DRIFT:
                rep.fail(f"高度偏差 {drift:.2f} m 超過 {MAX_ALT_DRIFT} —— "
                         "定高沒鎖住，或 controller 的加速度上限太大")
            else:
                rep.ok(f"高度全程維持在 ±{drift:.2f} m")

            worst, at = 1e9, None
            for px, py, _ in flyer.track:
                c = wall_clearance(px, py, walls)
                if c < worst:
                    worst, at = c, (px, py)
            print(f"     離牆最近：{worst:.2f} m（在 {at[0]:.1f}, {at[1]:.1f}）")
            if worst < MIN_FLIGHT_CLEARANCE:
                rep.fail(f"飛行中離牆只有 {worst:.2f} m"
                         f"（要 ≥ {MIN_FLIGHT_CLEARANCE}）—— 避障沒發揮作用")
            else:
                rep.ok(f"全程離牆 ≥ {worst:.2f} m")

            path_len = sum(
                math.hypot(flyer.track[i][0] - flyer.track[i - 1][0],
                           flyer.track[i][1] - flyer.track[i - 1][1])
                for i in range(1, len(flyer.track)))
            print(f"     實際飛行距離 {path_len:.1f} m")

        # ---------------- C5 有沒有被失效保護接管 ----------------
        rep.section("C6 翻覆診斷：傾角 vs 假回波")
        if not flyer.tilts:
            rep.warn("沒收到 vehicle_attitude")
        else:
            mx = max(t[3] for t in flyer.tilts)
            print(f"     最大合成傾角：{mx:.1f}°"
                  f"（{len(flyer.tilts)} 筆姿態樣本）")
            # 地板回波的距離 = 高度 / tan(傾角)
            if mx > 1:
                d = FLIGHT_ALT / math.tan(math.radians(mx))
                print(f"     該傾角下，朝下的光束會在 {d:.1f} m 處打到地板"
                      f"（obstacle_max_range 是 12 m）")
            if mx > 25:
                rep.fail(f"傾角到過 {mx:.1f}° —— 這已經是會翻的程度")
            elif mx > 15:
                rep.warn(f"傾角到過 {mx:.1f}°，地板回波會進入量程內")
            else:
                rep.ok(f"傾角最大 {mx:.1f}°，還算溫和")

        if not flyer.ghost_samples:
            rep.warn("沒取樣到光達資料")
        else:
            nfail = getattr(flyer, "tf_fail", 0)
            if nfail:
                print(f"     ⚠️ 有 {nfail} 次用掃描時間戳查 TF 失敗（樣本被跳過）")
            worst = max(flyer.ghost_samples, key=lambda t: t[1])
            avg = sum(t[1] for t in flyer.ghost_samples) / len(flyer.ghost_samples)
            print(f"     取樣 {len(flyer.ghost_samples)} 次，"
                  f"每次約 {flyer.ghost_samples[0][4]} 個回波納入比對")
            print(f"     不落在真牆上的回波：平均 {avg:.1f}%，"
                  f"最高 {worst[1]:.1f}%（當時傾角 {worst[0]:.1f}°）")
            if not math.isnan(worst[2]):
                print(f"     最高那次：光達在 map ({worst[5][0]:.2f}, "
                      f"{worst[5][1]:.2f})，最近的假回波 {worst[2]:.1f} m，"
                      f"平均 z={worst[3]:.2f}")
                print("     那次的假回波座標（x, y, z, 距離, 離最近的牆）：")
                for px_, py_, pz_, r_, c_ in worst[6]:
                    print(f"       ({px_:7.2f}, {py_:7.2f}, {pz_:5.2f})  "
                          f"r={r_:5.2f}  離牆 {c_:5.2f} m")
            ovd = getattr(flyer, "offset_vs_dist", [])
            if len(ovd) >= 5:
                print("     假回波的偏移量 vs 離起飛點的距離：")
                ovd.sort()
                for d_, o_ in ovd[::max(1, len(ovd) // 6)][:6]:
                    ang = math.degrees(math.atan2(o_, d_)) if d_ > 0 else 0
                    print(f"       離原點 {d_:5.1f} m  ->  偏移 {o_:5.2f} m"
                          f"  （等效角度誤差 {ang:4.1f}°）")
                angs = [math.degrees(math.atan2(o_, d_)) for d_, o_ in ovd if d_ > 0]
                spread = max(angs) - min(angs)
                mean_ang = sum(angs) / len(angs)
                print(f"     等效角度誤差：平均 {mean_ang:.1f}°，"
                      f"變化範圍 {spread:.1f}°")
                if spread < 4.0:
                    print("     → 角度幾乎是常數，代表這是「固定的航向誤差」，"
                          "不是光達雜訊")
                else:
                    print("     → 角度變化很大，不像單純的航向誤差")
            if avg > 10:
                rep.fail(f"平均有 {avg:.1f}% 的回波落在沒有牆的地方 —— "
                         "costmap 會出現假障礙物")
            else:
                rep.ok(f"假回波比例 {avg:.1f}%，在可接受範圍")

        rep.section("C6b TF 的位置 vs Gazebo 真值")
        # 前面所有的檢查都是「我們算的東西互相比」，永遠自洽。
        # 只有跟 Gazebo 的真值比，才知道 TF 本身有沒有偏掉。
        # 先讓飛機完全停穩再量 —— gz topic 要跑約一秒，
        # 期間飛機若還在轉，量到的「真值」和「PX4 的估計」就不是同一個時刻，
        # 會誤判成航向偏差。
        flyer.spin(4.0)
        samples = []
        for _ in range(3):
            t_ = flyer.gazebo_truth(env)
            if t_ is not None and t_[3] is not None and flyer.pos is not None:
                px4_enu_ = math.pi / 2.0 - flyer.pos.heading
                dy_ = px4_enu_ - t_[3]
                while dy_ > math.pi:
                    dy_ -= 2 * math.pi
                while dy_ < -math.pi:
                    dy_ += 2 * math.pi
                samples.append(math.degrees(dy_))
            flyer.spin(1.0)
        if samples:
            print(f"     航向誤差三次取樣："
                  f"{'、'.join(f'{v:+.1f}°' for v in samples)}"
                  f"（變化 {max(samples) - min(samples):.1f}°）")
        truth = flyer.gazebo_truth(env)
        if truth is None:
            rep.warn("讀不到 Gazebo 的真值位姿，跳過")
        else:
            tx, ty, _, tyaw = truth
            tf2_, err2_ = None, None
            try:
                import rclpy
                from rclpy.duration import Duration
                tf2_ = flyer.buf.lookup_transform(
                    "map", "base_link", rclpy.time.Time(),
                    timeout=Duration(seconds=2.0))
            except Exception as e:
                err2_ = str(e)
            if tf2_ is None:
                rep.fail(f"查不到 map -> base_link：{err2_[:60]}")
            else:
                bx = tf2_.transform.translation.x
                by = tf2_.transform.translation.y
                d = math.hypot(bx - tx, by - ty)
                print(f"     Gazebo 真值 : ({tx:.2f}, {ty:.2f})")
                print(f"     TF 算出來的 : ({bx:.2f}, {by:.2f})")
                print(f"     差距        : {d:.2f} m  "
                      f"（Δx {bx - tx:+.2f}  Δy {by - ty:+.2f}）")
                if d > 0.5:
                    rep.fail(f"TF 的位置和真值差 {d:.2f} m —— "
                             "光達的點會整體偏移，costmap 出現假障礙物")
                else:
                    rep.ok(f"TF 位置與真值一致（差 {d:.2f} m）")
                # 航向也比一下：PX4 的航向來自磁力計，位置來自 GPS，
                # 兩者若不一致，光達的方向就會整體轉一個角度。
                if tyaw is not None and flyer.pos is not None:
                    px4_enu = math.pi / 2.0 - flyer.pos.heading
                    while px4_enu > math.pi:
                        px4_enu -= 2 * math.pi
                    while px4_enu < -math.pi:
                        px4_enu += 2 * math.pi
                    dyaw = px4_enu - tyaw
                    while dyaw > math.pi:
                        dyaw -= 2 * math.pi
                    while dyaw < -math.pi:
                        dyaw += 2 * math.pi
                    print(f"     Gazebo 真值航向 : {math.degrees(tyaw):+.1f}°（ENU）")
                    print(f"     PX4 估的航向    : {math.degrees(px4_enu):+.1f}°（ENU）")
                    print(f"     航向誤差        : {math.degrees(dyaw):+.1f}°")
                    if abs(math.degrees(dyaw)) > 3:
                        rep.fail(f"PX4 的航向估計偏 {math.degrees(dyaw):+.1f}° —— "
                                 "光達的方向會整體轉這麼多")
                    else:
                        rep.ok(f"PX4 航向與真值一致（差 "
                               f"{math.degrees(dyaw):+.1f}°）")

        rep.section("C5 全程都由我們控制嗎")
        # nav_state 的變化史比 log 的字串可靠得多
        names = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
                 4: "AUTO_LOITER", 5: "AUTO_RTL", 10: "ACRO",
                 14: "OFFBOARD", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
                 20: "AUTO_PRECLAND"}
        hist = " -> ".join(names.get(n, str(n)) for n in flyer.nav_states)
        print(f"     nav_state 變化：{hist}")
        if flyer.battery is not None:
            print(f"     電池：{flyer.battery.remaining * 100:.0f}%"
                  f"  警告等級 {flyer.battery.warning}")
        f_ = flyer.failsafe
        if f_ is not None:
            hits = []
            for cond, msg in (
                    (f_.offboard_control_signal_lost, "offboard 訊號中斷"),
                    (f_.local_position_invalid, "本地位置估計失效"),
                    (f_.local_altitude_invalid, "高度估計失效"),
                    (f_.attitude_invalid, "姿態估計失效"),
                    (f_.battery_unhealthy, "電池不健康"),
                    (f_.battery_low_remaining_time, "電池剩餘時間不足")):
                if cond:
                    hits.append(msg)
            if f_.battery_warning != 0:
                hits.append(f"電池警告等級 {f_.battery_warning}")
            print(f"     failsafe 旗標：{'、'.join(hits) if hits else '（都正常）'}")

        bad_states = [n for n in flyer.nav_states if n in (5, 18, 20)]
        if bad_states:
            rep.fail(f"飛行途中被 PX4 接管（進了 {names.get(bad_states[0])}）—— "
                     "上面的飛行結果不能算數")
        elif 14 not in flyer.nav_states:
            rep.warn("整段都沒進過 OFFBOARD，Nav2 可能根本沒控制到飛機")
        else:
            rep.ok("全程由 offboard 控制，沒有被 PX4 接管")

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
