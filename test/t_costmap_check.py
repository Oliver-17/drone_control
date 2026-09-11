#!/usr/bin/env python3
# =============================================================================
#  t_costmap_check.py — Nav2 costmap 的驗證（S4）
#
#  用法：
#      python3 src/drone_control/test/t_costmap_check.py
#      python3 src/drone_control/test/t_costmap_check.py --gui
#
#  回傳值：全部通過 0，有任何一項失敗 1。
#
#  ⚠️ 會開 Gazebo（用 drone_nav2_apriltag 的場地），但「不會起飛」——
#     牆有 15 m 高，從地面就看得到，驗 costmap 不需要飛。
#     （上一支測試就是因為在這個場地裡飛才撞牆墜毀的。）
#
#  為什麼需要這支：
#      costmap 錯了，後面的 planner 和 controller 全部會錯，而三層的症狀
#      都是「飛得很怪」，分不開。這支把最底層單獨釘死。
#
#      而且 costmap 有兩個「靜默失敗」特別容易中：
#        1. 光達的 frame（lidar_link）沒有 TF -> 障礙物層完全沒資料
#        2. max_obstacle_height 預設 2.0 m，而無人機的光達裝在飛行高度
#           -> 打到牆的點在世界座標超過 2 m，被整批濾掉
#      兩者都不會讓 Nav2 報錯，只會讓它「看不到牆」然後規劃出穿牆的路。
#
#  它抓不到什麼：
#      - 規劃器選路對不對（S5）
#      - 控制器追得準不準（S6）
#      - 飛行中傾斜造成的光達誤判（要飛才會出現）
# =============================================================================

import argparse
import math
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

NAMESPACE = "MAV1"
INSTANCE = 0
MODEL = "x500_nav2"
WORLD = "nav2_arena"
SPAWN_X, SPAWN_Y = -2.0, 0.0   # 拓樸圖的節點 0，起飛區 A
SPAWN = f"{SPAWN_X},{SPAWN_Y},0,0,0,0"

# costmap 的 OccupancyGrid 數值（costmap_2d_publisher.hpp:176 的轉換表）：
#   100 = 障礙物本體    99 = 禁區（機器人中心進去會撞）
#   1~98 = 膨脹層       0 = 自由      -1 = 未知
OCC_LETHAL = 100
OCC_INSCRIBED = 99


class Report:
    def __init__(self):
        self.failed = 0
        self.warned = 0

    def section(self, t):
        print(f"\n── {t} " + "─" * max(0, 58 - len(t)))

    def ok(self, m):
        print(f"   ✓ {m}")

    def warn(self, m):
        print(f"   ! {m}")
        self.warned += 1

    def fail(self, m):
        print(f"   ✗ {m}")
        self.failed += 1


class Procs:
    """統一管理子程序。絕不用 pkill -f 寬鬆比對（會誤殺自己的 shell）。"""

    def __init__(self):
        self.items = []

    def start(self, name, cmd, env=None, log=None, cwd=None):
        f = open(log, "wb") if log else subprocess.DEVNULL
        p = subprocess.Popen(cmd, env=env, cwd=cwd, stdout=f,
                             stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        self.items.append((name, p, f))
        return p

    def stop_all(self):
        for _, p, _ in reversed(self.items):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass
        end = time.time() + 6
        while time.time() < end and any(p.poll() is None for _, p, _ in self.items):
            time.sleep(0.2)
        for _, p, f in self.items:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
            if hasattr(f, "close"):
                try:
                    f.close()
                except Exception:
                    pass
        for comm in ("px4", "ruby", "nav2_costmap_2d", "map_server",
                     "lifecycle_manager", "px4_tf_node", "cmd_vel_to_px4",
                     "static_transform_publisher", "parameter_bridge",
                     "image_bridge"):
            subprocess.run(["pkill", "-x", comm], capture_output=True)
        time.sleep(1)
        left = []
        for comm in ("px4", "ruby", "nav2_costmap_2d", "map_server"):
            r = subprocess.run(["pgrep", "-x", comm],
                               capture_output=True, text=True).stdout.split()
            left += [f"{comm}:{x}" for x in r]
        print(f"\n   收工：殘留 = {'有！' + ' '.join(left) if left else '無'}")
        return not left


def locate_arena():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = get_package_share_directory("drone_nav2_apriltag")
        if os.path.isfile(os.path.join(d, "gz", "worlds", f"{WORLD}.sdf")):
            return d
    except Exception:
        pass
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        c = os.path.join(d, "src", "drone_nav2_apriltag")
        if os.path.isfile(os.path.join(c, "gz", "worlds", f"{WORLD}.sdf")):
            return c
        d = os.path.dirname(d)
    return None


def parse_walls(world_sdf):
    """從 .sdf 撈出所有方塊障礙物（名稱, cx, cy, sx, sy, yaw度）。

    只認 <box>。ground_plane 用的是 <plane>（無限大），當成障礙物的話
    整張圖都是牆，所以自然被排除。
    """
    root = ET.parse(world_sdf).getroot()
    walls = []
    for model in root.iter("model"):
        pose = [float(v) for v in
                (model.findtext("pose") or "0 0 0 0 0 0").split()]
        pose += [0.0] * (6 - len(pose))
        for link in model.iter("link"):
            for col in list(link.iter("collision")) or list(link.iter("visual")):
                box = col.find("./geometry/box/size")
                if box is None:
                    continue
                sx, sy, _ = [float(v) for v in box.text.split()]
                walls.append((model.get("name"), pose[0], pose[1], sx, sy,
                              math.degrees(pose[5])))
                break
    return walls


def point_in_wall(px, py, cx, cy, sx, sy, yaw_deg, grow=0.0):
    a = math.radians(yaw_deg)
    dx, dy = px - cx, py - cy
    lx = dx * math.cos(a) + dy * math.sin(a)
    ly = -dx * math.sin(a) + dy * math.cos(a)
    return abs(lx) <= sx / 2.0 + grow and abs(ly) <= sy / 2.0 + grow


def set_sitl_params(build, env, instance, rep):
    """設定 SITL 的預檢與電池參數，並「逐一讀回確認」。

    為什麼一定要讀回：
        這兩組參數設失敗的症狀完全不像參數問題 ——
        NAV_DLL_ACT 沒設 -> 解鎖被擋（還算明顯）；
        SIM_BAT_DRAIN 沒設 -> 飛到一分鐘電池耗盡 -> 失效保護 -> RTL，
        飛機自己爬高飛回起點，看起來像「座標算錯」或「控制器發散」。
        這個坑在 t_bridge_check 和 t_nav2_flight_check 各踩過一次。

    ⚠️ 比對不能用字串包含：值是 "0" 的時候去掉尾零會變成空字串，
       而空字串永遠是任何字串的子字串，檢查會永遠通過。（也犯過。）
    """
    param = os.path.join(build, "bin", "px4-param")
    wanted = (
        # 機型檔 4001 會 set-default NAV_DLL_ACT 2，沒開 QGC 就 ARM 不起來
        ("NAV_DLL_ACT", "0"),
        ("CBRK_SUPPLY_CHK", "894281"),
        # 預設 60 秒就把電池從 100% 耗到 SIM_BAT_MIN_PCT
        ("SIM_BAT_DRAIN", "86400"),
        ("SIM_BAT_MIN_PCT", "99"),
        ("COM_LOW_BAT_ACT", "0"),
        # ⚠️ offboard 訊號中斷的判定時間，預設只有 1 秒。
        #    我們的架構是「誰要控制誰就發 cmd_vel」，節點交接時
        #    （起飛工具退場 -> Nav2 接手）一定會有一兩秒空窗，
        #    1 秒的話那個空窗就會被判定成失聯。
        #    實測症狀：OFFBOARD -> AUTO_RTL -> OFFBOARD，飛機自己爬高飛回起點，
        #    而 failsafe 旗標事後看又是正常的（因為已經恢復了）——
        #    非常難查。
        ("COM_OF_LOSS_T", "10.0"),
        # 真的失聯時的動作。預設是切回 Position（需要遙控器），
        # 5 = Hold（原地懸停）對伴飛電腦的架構最合理：
        # ROS 節點掛掉時飛機停在原地等人接手，而不是自己飛走。
        ("COM_OBL_RC_ACT", "5"),
        # ⚠️ 磁偏角。這是「光達在空地上畫出假牆」的根因。
        #
        #    SITL 裡 PX4 的「位置」來自模擬 GPS（對齊 Gazebo 世界），
        #    但「航向」來自磁力計。世界檔的磁場是
        #        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        #    （ENU：東、北、上），磁偏角 = atan2(6e-6, 2.3e-5) = 14.62°。
        #    PX4 預設用經緯度查表估磁偏角（蘇黎世約 3°），跟世界檔對不上，
        #    航向就差了十幾度 —— 位置對、航向錯，光達的點整體轉一個角度，
        #    空曠處就出現假牆。
        #
        #    EKF2_DECL_TYPE=0：不要查表，用下面這個值。
        #    （試過 EKF2_MAG_TYPE=5 完全關掉磁力計，但那樣航向在靜止時
        #      不可觀測，預檢會擋下解鎖：Arming denied: Resolve system
        #      health failures first。）
        ("EKF2_DECL_TYPE", "0"),
        ("EKF2_MAG_DECL", "14.62"),
        # ⚠️ 明確設回 0（Automatic）。
        #    px4-param save 是「持久化」的 —— 寫進參數檔之後會跨重啟保留。
        #    所以「把某個參數從這份清單移除」並不會把它改回預設值，
        #    上一次實驗設的值會一直留著。這個坑讓我以為新的修法失敗了
        #    （其實是舊的 EKF2_MAG_TYPE=5 還在，預檢照樣擋解鎖）。
        ("EKF2_MAG_TYPE", "0"),
    )
    for k, v in wanted:
        subprocess.run([param, "--instance", str(instance), "set", k, v],
                       capture_output=True, env=env)
    subprocess.run([param, "--instance", str(instance), "save"],
                   capture_output=True, env=env)

    bad = []
    for k, v in wanted:
        out = subprocess.run([param, "--instance", str(instance), "show", k],
                             capture_output=True, text=True, env=env).stdout
        got = None
        for tok in out.replace(":", " ").split():
            try:
                got = float(tok)
            except ValueError:
                continue
        if got is None or abs(got - float(v)) > 1e-6:
            bad.append(f"{k}: 想要 {v}，讀回 {got}")
    if bad:
        rep.fail("SITL 參數沒設進去：" + "；".join(bad)
                 + "。不修的話飛到一半會自己 RTL，所有飛行結果都不算數")
        return False
    rep.ok("預檢與電池參數已設定並逐一讀回確認")
    return True


def build_env(px4_dir, arena, headless):
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"
    envsh = os.path.join(px4_dir, "build", "px4_sitl_default", "rootfs", "gz_env.sh")
    out = subprocess.run(["bash", "-c", f"source {envsh} && env"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if k.startswith(("PX4_GZ_", "GZ_SIM_")):
                env[k] = v
    env["PX4_GZ_WORLDS"] = os.path.join(arena, "gz", "worlds")
    env["PX4_GZ_WORLD"] = WORLD
    env["GZ_SIM_RESOURCE_PATH"] = (os.path.join(arena, "gz", "models") + ":"
                                   + env.get("GZ_SIM_RESOURCE_PATH", ""))
    env["PX4_GZ_MODELS"] = os.path.join(arena, "gz", "models")
    if headless:
        env["HEADLESS"] = "1"
    return env


class Probe:
    def __init__(self, node):
        import tf2_ros
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid
        from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)

        self.node = node
        self.maps = {}
        self.scan = None

        # costmap 和 map 都是 transient_local（latched）——
        # 用預設 QoS 訂閱會收不到「在我們啟動之前就發出的那一筆」。
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 區域 costmap 由 controller_server 帶起來，留到 S6 才驗
        for key, topic in (("map", "/map"),
                           ("global", "/global_costmap/costmap")):
            node.create_subscription(
                OccupancyGrid, topic,
                lambda m, k=key: self.maps.__setitem__(k, m), latched)

        node.create_subscription(
            LaserScan, f"/{NAMESPACE}/scan",
            lambda m: setattr(self, "scan", m),
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE))

        # 懸停用：光達停在地上只離地 0.23 m，機身傾斜 2 度在 5 m 處就打到地板，
        # 一半的回波會變成「地板當成牆」。飛到 3 m 懸停才量得準。
        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.VOLATILE)
        self.pos = None
        self.status = None
        node.create_subscription(VehicleLocalPosition,
                                 f"/{NAMESPACE}/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "pos", m), best)
        node.create_subscription(VehicleStatus,
                                 f"/{NAMESPACE}/fmu/out/vehicle_status_v1",
                                 lambda m: setattr(self, "status", m), best)
        self.cmd_pub = node.create_publisher(Twist, f"/{NAMESPACE}/cmd_vel", 10)
        self.vc_pub = node.create_publisher(
            VehicleCommand, f"/{NAMESPACE}/fmu/in/vehicle_command", 10)
        self.VehicleCommand = VehicleCommand
        self.VehicleStatus = VehicleStatus
        self.Twist = Twist

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, node)

    def hold(self, seconds, rate_hz=20.0):
        """發零速度的 cmd_vel（定高懸停），並跑 ROS 迴圈。

        一定要限速 —— spin_once 幾乎立刻返回，直接在迴圈裡發等於用數千 Hz 灌，
        PX4 的佇列會塞爆，症狀是「解鎖了也進 offboard 了，就是不動」。
        """
        import rclpy
        m = self.Twist()
        end = time.time() + seconds
        period, nxt = 1.0 / rate_hz, 0.0
        while time.time() < end:
            t = time.time()
            if t >= nxt:
                self.cmd_pub.publish(m)
                nxt = t + period
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = self.VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = INSTANCE + 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vc_pub.publish(m)

    def spin(self, seconds):
        import rclpy
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, keys, timeout=60.0):
        import rclpy
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(k in self.maps for k in keys):
                return True
        return False

    def lookup(self, target, source, stamp=None):
        """查 TF。stamp 給 None 就取最新的。

        ⚠️ 拿「最新的 TF」去配「某一幀的感測器資料」會有時間差。
        懸停時飛機會慢慢飄，0.3 秒的時間差就是好幾公分 ——
        在比對光達回波落不落在牆上時，那個誤差會被放大成「整排點都差一點」。
        所以比對感測器資料時一定要用「該幀的時間戳」。
        """
        import rclpy
        from rclpy.duration import Duration
        try:
            t = rclpy.time.Time() if stamp is None \
                else rclpy.time.Time.from_msg(stamp)
            return self.buffer.lookup_transform(
                target, source, t, timeout=Duration(seconds=2.0)), None
        except Exception as e:
            return None, str(e)


def grid_at(grid, wx, wy):
    """查世界座標 (wx, wy) 在 OccupancyGrid 上的值，超出範圍回 None。

    OccupancyGrid 的 data 是「列優先、第 0 列在 origin 那一側」，
    也就是 y 最小的那一邊 —— 和 PGM（第 0 列在 y 最大）剛好相反。
    這兩個搞混的話地圖會上下顛倒，而顛倒的地圖看起來「也像一張地圖」。
    """
    info = grid.info
    col = int((wx - info.origin.position.x) / info.resolution)
    row = int((wy - info.origin.position.y) / info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return None
    return grid.data[row * info.width + col]


def main():
    ap = argparse.ArgumentParser(description="Nav2 costmap 驗證（開 Gazebo，不飛）")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    args = ap.parse_args()

    rep = Report()
    procs = Procs()
    scratch = f"/tmp/tcostmap_{os.getpid()}"
    os.makedirs(scratch, exist_ok=True)
    node = None

    try:
        rep.section("C0 環境")
        arena = locate_arena()
        if not arena:
            rep.fail("找不到 drone_nav2_apriltag")
            return 1
        rep.ok(f"場地包：{arena}")
        world_sdf = os.path.join(arena, "gz", "worlds", f"{WORLD}.sdf")
        walls = parse_walls(world_sdf)
        rep.ok(f"world 裡有 {len(walls)} 個方塊障礙物")

        build = os.path.join(args.px4_dir, "build", "px4_sitl_default")
        if not os.path.isfile(os.path.join(build, "bin", "px4")):
            rep.fail("找不到 px4 執行檔")
            return 1

        for c in ("px4", "ruby", "nav2_costmap_2d", "map_server",
                  "lifecycle_manager", "px4_tf_node"):
            subprocess.run(["pkill", "-x", c], capture_output=True)
        time.sleep(2)
        env = build_env(args.px4_dir, arena, headless=not args.gui)

        # ---- 啟動 ----
        rep.section("C1 啟動（不會起飛）")
        if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"],
                          capture_output=True).returncode != 0:
            procs.start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888"],
                        env=env, log=f"{scratch}/agent.log")
            time.sleep(2)
        rep.ok("MicroXRCEAgent 就緒")

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
        rep.ok("Gazebo 已啟動（飛機 spawn 在節點 0）")
        time.sleep(8)

        if not set_sitl_params(build, env, INSTANCE, rep):
            return 1

        # ⚠️ 一定要告訴 px4_tf_node「飛機開機的位置在 map 的哪裡」。
        #    PX4 的 local 原點是起飛點，map 的原點是世界原點，
        #    不補這個偏移的話 costmap 裡的牆會跟光達掃到的牆整體錯開。
        procs.start("bridge", ["ros2", "launch", "drone_control",
                               "px4_bridge.launch.py", f"namespace:={NAMESPACE}",
                               f"odom_origin:={SPAWN_X},{SPAWN_Y},0"],
                    env=env, log=f"{scratch}/bridge.log")
        procs.start("sensors", ["ros2", "launch", "drone_nav2_apriltag",
                               "cameras.launch.py", f"namespace:={NAMESPACE}",
                               "view:=false", "lidar:=true"],
                    env=env, log=f"{scratch}/sensors.log")
        time.sleep(5)
        procs.start("costmap", ["ros2", "launch", "drone_control",
                               "nav2.launch.py", f"namespace:={NAMESPACE}"],
                    env=env, log=f"{scratch}/costmap.log")
        rep.ok("px4_bridge / 感測器橋接 / costmap 都已啟動")

        import rclpy
        rclpy.init()
        # 用 sim time，才能跟橋接出來的感測器時間戳對齊
        node = rclpy.create_node(
            "t_costmap_check",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        probe = Probe(node)

        if not probe.wait_for(["map", "global"], timeout=90.0):
            missing = [k for k in ("map", "global") if k not in probe.maps]
            rep.fail(f"90 秒內收不到：{missing}。"
                     "看 costmap.log 的 lifecycle_manager 訊息")
            return 1
        rep.ok("已收到 /map 與 /global_costmap/costmap")
        probe.spin(3.0)
        return run_checks(rep, probe, walls, scratch) or (1 if rep.failed else 0)

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


def run_checks(rep, probe, walls, scratch):
    import json

    # ---------------- C2 TF 鏈 ----------------
    rep.section("C2 TF 鏈（costmap 全靠它）")
    for parent, child in (("map", "odom"), ("odom", "base_link"),
                          ("base_link", "lidar_link"), ("map", "lidar_link")):
        tf, err = probe.lookup(parent, child)
        if tf is None:
            hint = ""
            if child == "lidar_link":
                hint = ("（感測器的靜態 TF 由 cameras.launch.py 發，"
                        "少了它 costmap 會丟掉整層光達資料）")
            rep.fail(f"{parent} -> {child} 查不到：{err[:70]}{hint}")
        else:
            t = tf.transform.translation
            rep.ok(f"{parent} -> {child}  ({t.x:+.2f}, {t.y:+.2f}, {t.z:+.2f})")

    # ---------------- C2b map 系對得上世界嗎 ----------------
    # 這一項是為了抓一個很隱蔽的錯：map 和 odom 的原點不同。
    #
    # odom 原點 = PX4 的 EKF 初始化位置 = 飛機開機的地方
    # map  原點 = 靜態地圖的原點 = Gazebo 世界原點
    #
    # 把 map->odom 當成單位變換的話，所有 TF 仍然「彼此自洽」，
    # 拿 TF 跟 PX4 的位置對照也永遠一致 —— t_bridge_check 就是這樣所以沒抓到。
    # 只有拿「已知的世界座標」來比才看得出來。
    rep.section("C2b map 座標系對得上 Gazebo 世界嗎")
    tf, err = probe.lookup("map", "base_link")
    if tf is None:
        rep.fail(f"查不到 map -> base_link：{err[:70]}")
    else:
        t = tf.transform.translation
        # 飛機 spawn 在 (SPAWN_X, SPAWN_Y) 就沒再移動過（還沒起飛），
        # 所以 map 座標應該就是那個位置。
        dx, dy = t.x - SPAWN_X, t.y - SPAWN_Y
        d = math.hypot(dx, dy)
        print(f"     spawn 的世界座標 : ({SPAWN_X:+.2f}, {SPAWN_Y:+.2f})")
        print(f"     TF map->base_link: ({t.x:+.2f}, {t.y:+.2f})")
        if d > 0.5:
            rep.fail(f"差了 {d:.2f} m —— px4_tf_node 的 odom_origin_in_map "
                     "沒設或設錯。症狀會是「costmap 的牆和光達掃到的牆整體錯開」，"
                     "而所有 TF 看起來都正常")
        else:
            rep.ok(f"map 座標與世界一致（差 {d:.2f} m）")

    # ---------------- C3 靜態地圖 ----------------
    rep.section("C3 /map（map_server 載入的靜態地圖）")
    m = probe.maps["map"]
    rep.ok(f"{m.info.width} x {m.info.height} 格 @ {m.info.resolution:.2f} m，"
           f"原點 ({m.info.origin.position.x:.2f}, {m.info.origin.position.y:.2f})")

    # ---------------- C4 全域 costmap ----------------
    rep.section("C4 全域 costmap vs world 的牆")
    g = probe.maps["global"]
    info = g.info
    rep.ok(f"{info.width} x {info.height} 格 @ {info.resolution:.2f} m，"
           f"frame = {g.header.frame_id}")

    if g.header.frame_id != "map":
        rep.fail(f"全域 costmap 的 frame 是 {g.header.frame_id}，應該是 map")

    # 牆的中心應該是 lethal；離所有牆很遠的地方應該是自由
    wall_hits = wall_miss = 0
    for name, cx, cy, sx, sy, yaw in walls:
        v = grid_at(g, cx, cy)
        if v is None:
            continue
        if v >= OCC_INSCRIBED:
            wall_hits += 1
        else:
            wall_miss += 1
            rep.fail(f"牆 {name} 的中心 ({cx:.1f},{cy:.1f}) 在 costmap 上是 {v}，"
                     "應該是障礙物 —— 靜態地圖的原點或解析度可能對不上")
    if wall_miss == 0:
        rep.ok(f"{wall_hits} 面牆的中心在 costmap 上都是障礙物")

    # 膨脹層：牆外側應該有一圈「中間成本」的格子。
    #
    # ⚠️ 取樣距離要落在 robot_radius 和 inflation_radius 之間。
    #    太近（< robot_radius）會是 99（禁區），那不算膨脹；
    #    太遠（> inflation_radius）會是 0。
    #    之前取 0.3 m 就是落在 robot_radius 0.5 以內，所以一格都找不到，
    #    看起來像「inflation_layer 沒啟用」—— 其實是量錯地方。
    found_inflation = 0
    samples = 0
    for name, cx, cy, sx, sy, yaw in walls:
        a = math.radians(yaw)
        half = min(sx, sy) / 2.0
        for extra in (0.6, 0.7, 0.8, 0.9):
            off = half + extra
            for sign in (1, -1):
                if sy < sx:
                    px = cx - sign * off * math.sin(a)
                    py = cy + sign * off * math.cos(a)
                else:
                    px = cx + sign * off * math.cos(a)
                    py = cy + sign * off * math.sin(a)
                v = grid_at(g, px, py)
                if v is None:
                    continue
                samples += 1
                if 0 < v < OCC_INSCRIBED:
                    found_inflation += 1
    print(f"     取樣 {samples} 處（牆外 0.6~0.9 m），"
          f"其中 {found_inflation} 處是中間成本")
    if found_inflation == 0:
        rep.fail("牆外側找不到任何「中間成本」的格子 —— "
                 "inflation_layer 可能沒啟用，規劃器會貼著牆畫路徑")
    else:
        rep.ok(f"牆外側有 {found_inflation} 處膨脹成本（規劃器會自動遠離牆）")

    # 拓樸節點必須是低成本
    graph = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "..", "drone_nav2_apriltag",
        "graphs", "nav2_arena.geojson")
    nodes = {}
    try:
        from ament_index_python.packages import get_package_share_directory
        graph = os.path.join(get_package_share_directory("drone_nav2_apriltag"),
                             "graphs", "nav2_arena.geojson")
    except Exception:
        pass
    if os.path.isfile(graph):
        for f in json.load(open(graph))["features"]:
            p = f.get("properties", {})
            if "id" in p and f["geometry"]["type"] == "Point":
                nodes[p["id"]] = f["geometry"]["coordinates"]
    if nodes:
        bad = []
        for nid, (nx, ny) in sorted(nodes.items()):
            v = grid_at(g, nx, ny)
            if v is None or v >= OCC_INSCRIBED:
                bad.append(f"節點 {nid}={v}")
        if bad:
            rep.fail("這些拓樸節點在 costmap 上不可通行：" + "、".join(bad)
                     + " —— 規劃器會直接拒絕以它們為起點或終點")
        else:
            worst = max((grid_at(g, x, y) or 0) for x, y in nodes.values())
            rep.ok(f"{len(nodes)} 個拓樸節點都可通行（最高成本 {worst}）")
    else:
        rep.warn("找不到 geojson，跳過節點成本檢查")

    # ---------------- C5 光達有沒有真的進到 costmap ----------------
    # ⚠️ 必須先飛到 3 m 懸停才量得準。
    #    光達停在地上只離地 0.23 m，機身傾斜 2.4 度，光束在 5.5 m 處就打到地板 ——
    #    實測有一半的回波是「地板被當成牆」，那會讓這項檢查誤報失敗。
    #    懸停時只發零速度、原地不動，不會撞牆（節點 0 的淨空是 3.75 m）。
    rep.section("C5a 起飛懸停（為了把光達抬離地面）")
    probe.hold(2.0)
    probe.cmd(probe.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    probe.hold(0.5)
    probe.cmd(probe.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    probe.hold(1.0)
    for _ in range(10):
        st = probe.status
        armed = st is not None and st.arming_state == probe.VehicleStatus.ARMING_STATE_ARMED
        offb = st is not None and st.nav_state == probe.VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if armed and offb:
            break
        if not offb:
            probe.cmd(probe.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not armed:
            probe.cmd(probe.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        probe.hold(1.0)
    hovering = False
    if probe.status is not None and \
            probe.status.nav_state == probe.VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        end = time.time() + 40
        stable = 0
        while time.time() < end:
            probe.hold(0.5)
            if probe.pos and abs(-probe.pos.z - 3.0) < 0.2 and abs(probe.pos.vz) < 0.15:
                stable += 1
                if stable >= 4:
                    hovering = True
                    break
            else:
                stable = 0
    if hovering:
        rep.ok(f"已懸停在 {-probe.pos.z:.2f} m（光達離地約 {-probe.pos.z + 0.23:.2f} m）")
    else:
        rep.warn("沒能穩定懸停，光達仍在地面高度 —— "
                 "下面的回波檢查會混入地板回波，結果僅供參考")

    rep.section("C5b 光達資料（最容易靜默失敗的一環）")
    probe.spin(2.0)
    if probe.scan is None:
        rep.fail(f"收不到 /{NAMESPACE}/scan —— cameras.launch.py 的 lidar 沒開？")
    else:
        s = probe.scan
        finite = [r for r in s.ranges if s.range_min < r < s.range_max]
        rep.ok(f"收到 LaserScan：{len(s.ranges)} 點，"
               f"{len(finite)} 點有回波，frame = {s.header.frame_id}")
        if not finite:
            rep.fail("光達一個回波都沒有，場地裡應該要看得到牆")
        else:
            # 把回波點轉到 map 座標，檢查是不是落在某面牆上。
            # 這一步同時驗了「TF 對不對」和「光達量到的東西跟地圖一致」。
            # 先看時間戳在不在同一條時間軸上。
            # ros_gz_bridge 可能用 Gazebo 的 sim time（從 0 開始），
            # 而我們的節點用系統時間（1.78e9 那種）。兩者差很多的話，
            # 任何「用感測器時間戳查 TF」的動作都會失敗，
            # 而 Nav2 的 costmap 也會靜默地丟掉整層光達資料。
            now_s = probe.node.get_clock().now().nanoseconds / 1e9
            scan_s = s.header.stamp.sec + s.header.stamp.nanosec / 1e9
            skew = now_s - scan_s
            print(f"     時間戳：掃描 {scan_s:.1f}　節點 {now_s:.1f}　差 {skew:+.1f} s")
            if abs(skew) > 5.0:
                rep.fail(f"掃描的時間戳和節點差 {skew:+.1f} 秒 —— "
                         "兩邊不在同一條時間軸上（sim time vs 系統時間）。"
                         "Nav2 的 costmap 會因此丟掉整層光達資料")
            # 用這一幀掃描自己的時間戳查 TF。
            # 這同時也是「時間軸有沒有對齊」的證明 —— 查得到就代表
            # costmap 也轉換得了，查不到就代表整層光達資料會被丟掉。
            tf, err = probe.lookup("map", s.header.frame_id, s.header.stamp)
            if tf is None:
                rep.fail(f"用掃描的時間戳查不到 map -> {s.header.frame_id}："
                         f"{err[:60]} —— costmap 也會轉換失敗，"
                         "整層光達資料會被靜默丟掉")
            if tf is None:
                rep.fail(f"查不到 map -> {s.header.frame_id}：{err[:60]}")
            else:
                import math as _m
                # ⚠️ 不能只取 yaw 去做 2D 投影。
                #    懸停時機身會有幾度傾斜，光束就不在水平面上了 ——
                #    只用 yaw 投影會產生「跟距離成正比」的誤差
                #    （實測容差掃描：±0.1 m 只有 40%，±1.0 m 才 85%，
                #      那個形狀就是角度誤差的特徵）。
                #
                #    改成用完整的旋轉矩陣算 3D 方向，最後才取 x/y。
                #    牆有 15 m 高，光束打在牆的哪個高度不影響它的 (x,y) 落點，
                #    所以這樣算出來的水平位置是精確的。
                q = tf.transform.rotation
                qx, qy, qz, qw = q.x, q.y, q.z, q.w
                R = [
                    [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
                     2 * (qx * qz + qy * qw)],
                    [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
                     2 * (qy * qz - qx * qw)],
                    [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
                     1 - 2 * (qx * qx + qy * qy)],
                ]
                ox, oy = tf.transform.translation.x, tf.transform.translation.y
                # 先看回波距離的分佈。
                # 很近的那些幾乎一定是打到自己的機身（光達裝在 z=0.26，
                # 剛好在 x500 的機臂與馬達高度附近），不是真的障礙物。
                SELF_HIT = 0.6
                buckets = {"<0.6（機身）": 0, "0.6~3": 0, "3~10": 0, ">10": 0}
                for r in s.ranges:
                    if not (s.range_min < r < s.range_max):
                        continue
                    if r < SELF_HIT:
                        buckets["<0.6（機身）"] += 1
                    elif r < 3:
                        buckets["0.6~3"] += 1
                    elif r < 10:
                        buckets["3~10"] += 1
                    else:
                        buckets[">10"] += 1
                print("     回波距離分佈：" +
                      "  ".join(f"{k}={v}" for k, v in buckets.items()))

                pts = []
                for i, r in enumerate(s.ranges):
                    if not (SELF_HIT < r < s.range_max):
                        continue
                    a = s.angle_min + i * s.angle_increment
                    # 光束在 lidar 座標系裡的方向（水平面上）
                    vx, vy, vz = _m.cos(a), _m.sin(a), 0.0
                    # 轉到 map 座標系（完整 3D 旋轉，含傾斜）
                    mx = R[0][0] * vx + R[0][1] * vy + R[0][2] * vz
                    my = R[1][0] * vx + R[1][1] * vy + R[1][2] * vz
                    pts.append((ox + r * mx, oy + r * my, r))

                # 角度掃描：如果整批光束有固定的角度偏移，這裡會看到一個明顯的峰。
                # 峰不在 0 度就代表方向算錯了（frame 的旋轉、angle_min 的定義、
                # 或是掃描方向相反），而不是雜訊。
                best_d, best_pct = 0.0, -1.0
                rows = []
                for ddeg in range(-16, 17, 2):
                    d = _m.radians(ddeg)
                    cd, sd = _m.cos(d), _m.sin(d)
                    hit = 0
                    for i, r in enumerate(s.ranges):
                        if not (SELF_HIT < r < s.range_max):
                            continue
                        a = s.angle_min + i * s.angle_increment
                        vx, vy = _m.cos(a), _m.sin(a)
                        # 先在 lidar 系裡多轉 ddeg，再轉到 map
                        rx, ry = vx * cd - vy * sd, vx * sd + vy * cd
                        mx = R[0][0] * rx + R[0][1] * ry
                        my = R[1][0] * rx + R[1][1] * ry
                        px, py = ox + r * mx, oy + r * my
                        if any(point_in_wall(px, py, cx, cy, sx, sy, w, grow=0.3)
                               for _, cx, cy, sx, sy, w in walls):
                            hit += 1
                    pctd = hit / max(1, len([1 for r in s.ranges
                                             if SELF_HIT < r < s.range_max])) * 100
                    rows.append((ddeg, pctd))
                    if pctd > best_pct:
                        best_pct, best_d = pctd, ddeg
                print("     角度偏移掃描（多轉 N 度後落在牆上的比例）：")
                print("       " + "  ".join(f"{d:+d}°={p:.0f}%" for d, p in rows))
                if abs(best_d) > 2 and best_pct > pct_placeholder_unused if False else False:
                    pass
                print(f"     最佳偏移 {best_d:+d}° -> {best_pct:.1f}%")

                # 容差掃描：看誤差是「整體差一點」還是「真的算錯」。
                # 整體差一點（幾公分）通常是時間差或雜訊；
                # 差好幾公尺才是方向或偏移搞錯。
                print("     容差掃描（落在牆上的比例）：")
                table = {}
                for tol in (0.1, 0.2, 0.3, 0.5, 1.0):
                    hit = sum(1 for px, py, _ in pts
                              if any(point_in_wall(px, py, cx, cy, sx, sy, w, grow=tol)
                                     for _, cx, cy, sx, sy, w in walls))
                    table[tol] = hit / len(pts) * 100 if pts else 0
                    print(f"       ±{tol:.1f} m -> {table[tol]:5.1f}%")

                on_wall = sum(1 for px, py, _ in pts
                              if any(point_in_wall(px, py, cx, cy, sx, sy, w, grow=0.3)
                                     for _, cx, cy, sx, sy, w in walls))
                total = len(pts)
                pct = table[0.3]
                worst = []
                for px, py, r in pts:
                    if any(point_in_wall(px, py, cx, cy, sx, sy, w, grow=1.0)
                           for _, cx, cy, sx, sy, w in walls):
                        continue
                    if len(worst) < 6:
                        worst.append(f"({px:.1f},{py:.1f}) r={r:.1f}")
                if worst:
                    print(f"     連 ±1.0 m 都落不到牆上的：{', '.join(worst)}")
                else:
                    print("     ±1.0 m 之內所有回波都落在牆上")
                thr = 90 if hovering else 45
                if pct < thr:
                    rep.fail(f"只有 {pct:.1f}% 的回波落在牆上（門檻 {thr}%）。"
                             "TF 的角度或位置可能錯了，或光達打到了地板")
                else:
                    rep.ok(f"{pct:.1f}% 的回波落在牆上 —— 光達與地圖一致")
                if buckets["<0.6（機身）"] > 50:
                    rep.warn(f"有 {buckets['<0.6（機身）']} 個回波在 0.6 m 以內，"
                             "光達打到自己的機身。這些會被 costmap 當成障礙物標在"
                             "飛機腳邊 —— 要靠 ObstacleLayer 的最小距離或"
                             "model.sdf 的 range min 濾掉")

        # costmap 有沒有抱怨轉換失敗。這是 max_obstacle_height 或缺 TF 的典型徵兆，
        # 而 Nav2 只會印 WARN，不會讓任何檢查失敗 —— 所以要主動翻 log。
        log = os.path.join(scratch, "costmap.log")
        if os.path.isfile(log):
            with open(log, "r", errors="ignore") as f:
                txt = f.read()
            for pat, why in (("Could not transform", "TF 缺失或時間戳對不上"),
                             ("sensor origin", "光達位置超出 costmap 範圍"),
                             ("Lookup would require extrapolation",
                              "TF 時間戳落後")):
                n = txt.count(pat)
                if n:
                    rep.fail(f"costmap log 出現 {n} 次「{pat}」—— {why}")
            if not any(p in txt for p in ("Could not transform", "sensor origin")):
                rep.ok("costmap log 沒有 TF 或感測器相關的錯誤")

    # 區域 costmap（local_costmap）由 controller_server 帶起來，留到 S6 驗。

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過" + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
