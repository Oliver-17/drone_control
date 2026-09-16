#!/usr/bin/env python3
# =============================================================================
#  t_bridge_check.py — px4_tf_node / cmd_vel_to_px4_node 的座標驗證（S1 + S2）
#
#  用法：
#      python3 src/drone_control/test/t_bridge_check.py
#      python3 src/drone_control/test/t_bridge_check.py --gui
#
#  回傳值：全部通過 0，有任何一項失敗 1。
#
#  ⚠️ 這支會真的讓飛機飛起來（低速、小範圍）。跑之前確認沒有別的 SITL 在跑。
#
#  為什麼需要這支：
#      這兩支節點做的是「座標系翻譯」，而翻譯錯了「不會報錯」——
#      Nav2 會很認真地往錯的方向導航，飛機確實在動、確實在收斂，
#      只是收斂到錯的地方。這種錯只能靠「放在已知狀態，檢查算出來的答案對不對」抓。
#
#      Oliver 已經踩過兩次同類的坑：OptiTrack 的 NED 轉了 90 度、
#      AprilTag 的 ENU/NED 符號翻轉。這支就是為了不要再有第三次。
#
#  怎麼驗：
#      參考基準用 PX4 的 vehicle_local_position（NED，定義明確：x 北 / y 東 / z 下）。
#      在 SITL 裡它由 Gazebo 直接餵，等同真值。
#
#        C2  TF 的位置 == PX4 位置做 NED->ENU 之後的值
#        C3  TF 的航向 == PX4 heading 做 NED->ENU 之後的值
#        C4  發 cmd_vel，檢查飛機「往哪個方向動」符合預期  ← 只有這項要飛
#
#      C4 特別重要：只驗靜態的 C2/C3 的話，「機體座標轉世界座標」那一段
#      （要繞 heading 轉）完全沒被驗到 —— 而那正是 S2 最容易錯的地方。
#
#  它抓不到什麼（誠實說明）：
#      - PX4 自己的 NED 定義對不對（那是 PX4/Gazebo 的事，不是這兩支節點的）
#      - 實機的磁力計偏差造成的 heading 誤差
# =============================================================================

import argparse
import math
import os
import signal
import subprocess
import sys
import time

NAMESPACE = "MAV1"
INSTANCE = 0
# 用 PX4 內建的 x500 就好 —— 這支測試不需要相機或光達。
MODEL = "x500"
# ⚠️ 刻意用 PX4 內建的空世界，不用 drone_nav2_apriltag 的場地。
# 這支測試驗的是「座標轉換」，跟場地無關；而在有 15 m 高牆的場地裡飛，
# 飛機會撞牆墜毀（實測過：Attitude failure(roll) + Imbalanced propeller，
# 高度掉到地面以下），所有方向測試全部失準，而且看起來像轉換寫錯。
# 空世界也讓這支測試不再相依任何其他套件。
WORLD = "default"

# 位置比對的容許值（公尺）。這是純數學轉換，理論上誤差是 0，
# 放寬到 5 cm 純粹是為了容忍兩個 topic 的時間差（TF 用最新的姿態配最新的位置）。
POS_TOL = 0.05
YAW_TOL_DEG = 3.0

# C4 每個方向要移動多少才算「確實往那邊動了」（公尺）。
# 0.5 m 遠大於位置估計的雜訊，又小到不會撞牆。
MOVE_MIN = 0.5


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
    """統一管理子程序。

    絕對不要用 pkill -f "gz sim" 這種寬鬆比對 —— 它會連「命令列裡剛好出現
    這幾個字」的無關程序一起殺，包括呼叫這支腳本的 shell 自己。
    """

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
        for comm in ("px4", "ruby"):
            subprocess.run(["pkill", "-x", comm], capture_output=True)
        time.sleep(1)
        left = []
        for comm in ("px4", "ruby", "px4_tf_node", "cmd_vel_to_px4"):
            r = subprocess.run(["pgrep", "-x", comm], capture_output=True,
                               text=True).stdout.split()
            left += [f"{comm}:{x}" for x in r]
        print(f"\n   收工：殘留 = {'有！' + ' '.join(left) if left else '無'}")
        return not left


class Probe:
    """訂 PX4 的位置、查 TF，並且能發 cmd_vel 與 PX4 指令。"""

    def __init__(self, node, ns):
        import tf2_ros
        from geometry_msgs.msg import Twist
        from px4_msgs.msg import (BatteryStatus, FailsafeFlags, VehicleCommand,
                                  VehicleLocalPosition, VehicleStatus)
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)

        self.node = node
        self.VehicleCommand = VehicleCommand
        self.VehicleStatus = VehicleStatus
        self.Twist = Twist

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.pos = None
        self.status = None
        node.create_subscription(
            VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
            lambda m: setattr(self, "pos", m), qos)
        node.create_subscription(
            VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "status", m), qos)
        # 失效保護一旦觸發，飛機就不再聽我們的 setpoint 了 ——
        # 表現是「高度亂跳、往奇怪的方向去」，看起來完全像座標轉換寫錯。
        # 訂這個才能一眼看出是誰的問題。
        self.failsafe = None
        node.create_subscription(
            FailsafeFlags, f"/{ns}/fmu/out/failsafe_flags",
            lambda m: setattr(self, "failsafe", m), qos)
        # 電池是這支測試最常見的假失敗來源，直接看數字最實在
        self.battery = None
        node.create_subscription(
            BatteryStatus, f"/{ns}/fmu/out/battery_status",
            lambda m: setattr(self, "battery", m), qos)

        self.cmd_pub = node.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.vc_pub = node.create_publisher(
            VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, node)

    # ---- 基本工具 ----
    def spin(self, seconds, twist=None, rate_hz=20.0):
        """跑 ROS 迴圈，同時以固定頻率發 cmd_vel。

        ⚠️ 一定要限速。spin_once 在有訊息可處理時幾乎立刻返回，
        直接在迴圈裡發等於用數千 Hz 灌，PX4 端的佇列會塞爆 ——
        症狀是「解鎖了、也進 offboard 了，就是不動」。
        """
        import rclpy
        end = time.time() + seconds
        period = 1.0 / rate_hz
        nxt = 0.0
        while time.time() < end:
            t = time.time()
            if twist is not None and t >= nxt:
                self.cmd_pub.publish(twist)
                nxt = t + period
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def twist(self, vx=0.0, vy=0.0, wz=0.0):
        m = self.Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        return m

    def cmd(self, command, p1=0.0, p2=0.0):
        m = self.VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = INSTANCE + 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True   # 少了這個 PX4 會當成內部指令直接拒絕
        m.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vc_pub.publish(m)

    def lookup(self, target, source):
        import rclpy
        from rclpy.duration import Duration
        try:
            return self.buffer.lookup_transform(
                target, source, rclpy.time.Time(), timeout=Duration(seconds=1.0)), None
        except Exception as e:
            return None, str(e)

    def wait_position(self, timeout=90.0):
        import rclpy
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                return True
        return False


def failsafe_reason(f):
    """把 FailsafeFlags 翻譯成人看得懂的原因。回傳 None 代表一切正常。"""
    if f is None:
        return None
    checks = [
        (f.offboard_control_signal_lost, "offboard 訊號中斷（setpoint 沒有持續發）"),
        (f.local_position_invalid, "本地位置估計失效"),
        (f.local_altitude_invalid, "高度估計失效"),
        (f.attitude_invalid, "姿態估計失效"),
        (f.battery_unhealthy, "電池不健康"),
        (f.battery_low_remaining_time, "電池剩餘時間不足"),
    ]
    hits = [m for cond, m in checks if cond]
    if f.battery_warning != 0:
        hits.append(f"電池警告等級 {f.battery_warning}")
    return "、".join(hits) if hits else None


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def build_env(px4_dir, headless):
    env = os.environ.copy()
    # gz-transport 綁回環位址。不設的話 IMU 傳遞會抖 -> Accel TIMEOUT -> EKF 劣化。
    # （來源：drone_nav2_apriltag/scripts/start_arena_sitl.sh 的實測結論）
    env["GZ_IP"] = "127.0.0.1"
    envsh = os.path.join(px4_dir, "build", "px4_sitl_default", "rootfs", "gz_env.sh")
    out = subprocess.run(["bash", "-c", f"source {envsh} && env"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if k.startswith(("PX4_GZ_", "GZ_SIM_")):
                env[k] = v
    # 世界與機體都用 PX4 內建的，gz_env.sh 已經把路徑設好了，不用再覆蓋
    env["PX4_GZ_WORLD"] = WORLD
    if headless:
        env["HEADLESS"] = "1"
    return env





def main():
    ap = argparse.ArgumentParser(
        description="px4_tf_node / cmd_vel_to_px4_node 的座標驗證（會讓飛機飛）")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    ap.add_argument("--altitude", type=float, default=3.0)
    args = ap.parse_args()

    rep = Report()
    procs = Procs()
    scratch = f"/tmp/tbridge_{os.getpid()}"
    os.makedirs(scratch, exist_ok=True)
    node = None

    try:
        rep.section("C0 環境")
        rep.ok(f"世界：PX4 內建的 {WORLD}（空曠，不會撞牆）")
        rep.ok(f"機體：{MODEL}")

        build = os.path.join(args.px4_dir, "build", "px4_sitl_default")
        px4_bin = os.path.join(build, "bin", "px4")
        if not os.path.isfile(px4_bin):
            rep.fail(f"找不到 {px4_bin}")
            return 1

        # 這幾個名字都 <= 15 字元所以安全。超過 15 的要先截斷 ——
        # comm 的上限是 15，pkill -x 用全名會靜默失效（見 t_costmap_check
        # 的 pkill_exact 註解）。
        for c in ("px4", "ruby", "px4_tf_node", "cmd_vel_to_px4"):
            subprocess.run(["pkill", "-x", c], capture_output=True)
        time.sleep(2)

        env = build_env(args.px4_dir, headless=not args.gui)

        rep.section("C1 啟動模擬與兩支節點")
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
                     "PX4_SIM_MODEL": MODEL, "PX4_GZ_MODEL_POSE": "0,0,0,0,0,0"})
        procs.start("px4", [px4_bin, "-i", str(INSTANCE), "-d",
                            os.path.join(build, "etc")],
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
        time.sleep(8)   # 等 uxrce_dds_client 把 topic 註冊出去

        # 機型檔 4001 會 set-default NAV_DLL_ACT 2，沒開 QGC 就 ARM 不起來
        param = os.path.join(build, "bin", "px4-param")
        # SIM_BAT_DRAIN 預設 60 —— 60 秒就把模擬電池從 100% 耗到 SIM_BAT_MIN_PCT(50%)，
        # 這支測試會飛超過一分鐘，不改的話必定觸發電池失效保護 -> RTL，
        # 飛機會自己爬到 5 m 飛回起點，所有方向測試全部失準。
        # 症狀是「高度亂跳、往奇怪的方向去」，很容易誤判成自己的轉換寫錯。
        # SIM_BAT_DRAIN 預設 60 秒就把電池耗到 SIM_BAT_MIN_PCT(50%)，
        # 這支測試會飛好幾分鐘，不改的話必定觸發電池失效保護 -> RTL。
        # RTL 的表現是「自己爬高、往起點飛」，看起來完全像座標轉換寫錯 ——
        # 這個坑踩過兩次，所以下面會把電池狀態讀回來確認，不是設了就算。
        wanted = (("NAV_DLL_ACT", "0"), ("CBRK_SUPPLY_CHK", "894281"),
                  ("SIM_BAT_DRAIN", "86400"), ("SIM_BAT_MIN_PCT", "99"),
                  ("COM_LOW_BAT_ACT", "0"))
        for k, v in wanted:
            subprocess.run([param, "--instance", str(INSTANCE), "set", k, v],
                           capture_output=True, env=env)
        subprocess.run([param, "--instance", str(INSTANCE), "save"],
                       capture_output=True, env=env)

        # 逐一讀回。
        # ⚠️ 比對不能用「子字串包含」—— 值是 "0" 的時候，去掉尾零會變成空字串，
        #    而空字串永遠是任何字串的子字串，檢查會永遠通過。（這個 bug 犯過一次。）
        bad = []
        for k, v in wanted:
            out = subprocess.run([param, "--instance", str(INSTANCE), "show", k],
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
            rep.fail("參數沒設進去：" + "；".join(bad)
                     + "。不修的話飛到一半會自己 RTL，所有方向測試都會失準")
        else:
            rep.ok("預檢與電池參數已設定並逐一讀回確認")

        procs.start("bridge", ["ros2", "launch", "drone_control",
                               "px4_bridge.launch.py",
                               f"namespace:={NAMESPACE}",
                               f"flight_altitude:={args.altitude}"],
                    env=env, log=f"{scratch}/bridge.log")
        rep.ok("px4_tf_node + cmd_vel_to_px4_node 已啟動")
        time.sleep(4)

        import rclpy
        rclpy.init()
        node = rclpy.create_node("t_bridge_check")
        probe = Probe(node, NAMESPACE)

        if not probe.wait_position():
            rep.fail("收不到 vehicle_local_position")
            return 1
        probe.spin(2.0)

        # ---------------- C2 TF 位置 ----------------
        rep.section("C2 TF 位置（NED -> ENU）")
        tf, err = probe.lookup("odom", "base_link")
        if tf is None:
            rep.fail(f"查不到 odom -> base_link：{err}。px4_tf_node 沒在發 TF？")
            return 1
        p, t = probe.pos, tf.transform.translation
        # PX4 NED: x=北 y=東 z=下   ->   ROS ENU: x=東 y=北 z=上
        exp = (p.y, p.x, -p.z)
        d = max(abs(t.x - exp[0]), abs(t.y - exp[1]), abs(t.z - exp[2]))
        print(f"     PX4 NED  : 北 {p.x:+.3f}  東 {p.y:+.3f}  下 {p.z:+.3f}")
        print(f"     期望 ENU : 東 {exp[0]:+.3f}  北 {exp[1]:+.3f}  上 {exp[2]:+.3f}")
        print(f"     TF   ENU : 東 {t.x:+.3f}  北 {t.y:+.3f}  上 {t.z:+.3f}")
        if d > POS_TOL:
            rep.fail(f"位置對不上，最大差 {d:.3f} m。"
                     "多半是 x/y 沒交換，或 z 的符號沒反")
        else:
            rep.ok(f"位置一致（最大差 {d * 100:.1f} cm）")

        # ---------------- C3 TF 航向 ----------------
        rep.section("C3 TF 航向（NED heading -> ENU yaw）")
        tf_yaw = quat_to_yaw(tf.transform.rotation)
        # NED heading 從「北」起算、順時針為正；ENU yaw 從「東」起算、逆時針為正。
        # 同一個朝向：ENU_yaw = pi/2 - NED_heading
        exp_yaw = wrap_pi(math.pi / 2.0 - p.heading)
        dy = abs(wrap_pi(tf_yaw - exp_yaw))
        print(f"     PX4 heading : {math.degrees(p.heading):+.1f}°（從北，順時針）")
        print(f"     期望 ENU yaw: {math.degrees(exp_yaw):+.1f}°（從東，逆時針）")
        print(f"     TF   yaw    : {math.degrees(tf_yaw):+.1f}°")
        if dy > math.radians(YAW_TOL_DEG):
            rep.fail(f"航向差 {math.degrees(dy):.1f}°，四元數轉換有問題")
        else:
            rep.ok(f"航向一致（差 {math.degrees(dy):.1f}°）")
        return run_flight(rep, probe, args) or (1 if rep.failed else 0)

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


def run_flight(rep, probe, args):
    """C4：真的飛，驗 cmd_vel 的方向。

    只驗 C2/C3 是不夠的 —— 那兩項都是靜態的，
    「機體速度繞 heading 轉成世界速度」那一段完全沒被驗到，
    而那正是 cmd_vel_to_px4_node 最容易錯的地方。
    """
    rep.section("C4 cmd_vel 方向（會飛）")

    # 先讓 cmd_vel_to_px4_node 開始發 setpoint，PX4 才准切 offboard。
    # 順序反過來的話切模式指令會被拒絕，而且訊息很不明顯。
    zero = probe.twist()
    probe.spin(2.0, zero)

    probe.cmd(probe.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    probe.spin(0.5, zero)
    probe.cmd(probe.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    probe.spin(1.0, zero)

    for _ in range(10):
        s = probe.status
        armed = s is not None and s.arming_state == probe.VehicleStatus.ARMING_STATE_ARMED
        offb = s is not None and s.nav_state == probe.VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if armed and offb:
            break
        if not offb:
            probe.cmd(probe.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not armed:
            probe.cmd(probe.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        probe.spin(1.0, zero)
    else:
        rep.fail("解鎖或切 offboard 失敗，看 px4.log")
        return 1

    # 只發零速度就會自己爬升：cmd_vel_to_px4_node 把 z 設成位置 setpoint，
    # 水平設成速度 0 —— 這本身就是「定高懸停」的指令。
    b = probe.battery
    if b is not None:
        print(f"     電池：{b.remaining * 100:.0f}%  警告等級 {b.warning}")
        if b.remaining < 0.9:
            rep.fail(f"電池只剩 {b.remaining * 100:.0f}%，飛到一半會觸發失效保護。"
                     "SIM_BAT_DRAIN / SIM_BAT_MIN_PCT 沒生效")
            return 1
    rep.ok("已解鎖並進入 offboard，靠零速度 cmd_vel 自動爬升")
    end = time.time() + 40
    while time.time() < end:
        probe.spin(0.3, zero)
        if abs(-probe.pos.z - args.altitude) < 0.3:
            break
    else:
        rep.fail(f"爬升逾時（目前 {-probe.pos.z:.2f} m）")
        return 1
    # 到達高度不等於穩定。爬升結束時可能還帶著垂直速度，
    # 這時開始量會把「還在收斂的振盪」算進移動量，測出一堆假的失敗。
    rep.ok(f"到達 {args.altitude:.1f} m，等待穩定…")
    stable = 0
    end = time.time() + 30
    while time.time() < end:
        probe.spin(0.5, zero)
        alt_ok = abs(-probe.pos.z - args.altitude) < 0.15
        vz_ok = abs(probe.pos.vz) < 0.15
        stable = stable + 1 if (alt_ok and vz_ok) else 0
        if stable >= 6:      # 連續 3 秒都穩
            break
    if stable < 6:
        rep.warn(f"30 秒內沒完全穩定（高度 {-probe.pos.z:.2f} m，"
                 f"垂直速度 {probe.pos.vz:+.2f} m/s），下面的結果可能偏差")
    else:
        rep.ok(f"已穩定在 {-probe.pos.z:.2f} m，開始測方向")

    def move_test(label, vx, vy, seconds):
        """發一個 cmd_vel，檢查飛機實際往「該去的方向」動了。

        期望方向是從「當下的機頭」現算的，不是寫死「北」——
        飛機 spawn 的朝向不保證是正北（實測 x500 是朝東，NED heading +90°），
        寫死的話測試會在正確的實作上誤報失敗。

        推導（和 cmd_vel_to_px4_node 用的是同一條公式，但這裡獨立寫一次，
        故意不共用程式碼 —— 共用的話兩邊一起錯就驗不出來）：
            FRD 速度  = (vx, -vy_ros)          ROS 的 +y 是左，FRD 的 +y 是右
            北 = vx*cos(h) + vy_ros*sin(h)
            東 = vx*sin(h) - vy_ros*cos(h)
        """
        h = probe.pos.heading
        exp_n = vx * math.cos(h) + vy * math.sin(h)
        exp_e = vx * math.sin(h) - vy * math.cos(h)
        norm = math.hypot(exp_n, exp_e)
        if norm < 1e-6:
            return
        ux, uy = exp_n / norm, exp_e / norm

        p0 = (probe.pos.x, probe.pos.y, -probe.pos.z)
        probe.spin(seconds, probe.twist(vx=vx, vy=vy))
        probe.spin(2.0, zero)   # 停穩再量，避免帶著速度
        p1 = (probe.pos.x, probe.pos.y, -probe.pos.z)
        dn, de, dalt = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]

        why = failsafe_reason(probe.failsafe)
        if why:
            rep.fail(f"{label}：測試期間失效保護觸發（{why}）—— "
                     "飛機已經不聽我們的指令，這一項的結果無效")
            return

        along = dn * ux + de * uy        # 沿期望方向走了多遠
        perp = -dn * uy + de * ux        # 垂直於期望方向偏了多遠

        print(f"\n     [{label}]")
        print(f"       當下機頭 {math.degrees(h):+.1f}°  ->  "
              f"期望方向 北{ux:+.2f} 東{uy:+.2f}")
        print(f"       實際移動 北 {dn:+.2f}  東 {de:+.2f}  高度 {dalt:+.2f} m")
        print(f"       沿期望方向 {along:+.2f} m，側偏 {perp:+.2f} m")

        if along < MOVE_MIN:
            rep.fail(f"{label}：沿期望方向只走了 {along:+.2f} m（要 ≥ {MOVE_MIN}）。"
                     "方向錯了 —— 檢查機體 FLU->FRD->NED 那條鏈")
        elif abs(perp) > along * 0.4:
            rep.fail(f"{label}：側偏 {perp:+.2f} m，超過前進量的 40%。"
                     "heading 旋轉可能算錯")
        else:
            rep.ok(f"{label}：沿期望方向 {along:+.2f} m，側偏僅 {perp:+.2f} m")

        if abs(dalt) > 0.3:
            rep.fail(f"{label}：高度跑掉 {dalt:+.2f} m，定高沒鎖住")

    print(f"\n     起飛後機頭：{math.degrees(probe.pos.heading):+.1f}°"
          "（0=北 90=東，x500 spawn 時朝東是正常的）")

    move_test("前進 linear.x=+0.6", 0.6, 0.0, 4.0)
    move_test("左移 linear.y=+0.6", 0.0, 0.6, 4.0)
    move_test("後退 linear.x=-0.6", -0.6, 0.0, 4.0)

    # ③ 轉向 90 度後再前進 —— 這一項專門驗「繞 heading 旋轉」那一段。
    #    不轉的話 heading 一直是 0，cos/sin 都是常數，公式寫錯也看不出來。
    rep.section("C4b 轉向後再前進（驗 heading 旋轉）")
    # ROS 的 angular.z 逆時針為正；要讓機頭從北轉到東（NED heading +90°），
    # 在 ENU 看是順時針，所以 angular.z 要給負值。
    target = wrap_pi(probe.pos.heading + math.radians(90.0))
    end = time.time() + 25
    while time.time() < end:
        errh = wrap_pi(target - probe.pos.heading)
        if abs(errh) < math.radians(5):
            break
        probe.spin(0.2, probe.twist(wz=-0.5 if errh > 0 else 0.5))
    probe.spin(2.0, zero)
    h2 = math.degrees(probe.pos.heading)
    print(f"     轉向後機頭：{h2:+.1f}°（目標 {math.degrees(target):+.1f}°）")
    if abs(wrap_pi(probe.pos.heading - target)) > math.radians(15):
        rep.fail(f"轉向失敗，機頭在 {h2:+.1f}°。"
                 "angular.z 的正負可能反了（ROS 逆時針為正，PX4 順時針為正）")
    else:
        rep.ok(f"機頭已轉到 {h2:+.1f}°")
        # 轉了 90 度之後再前進：期望方向也跟著轉 90 度。
        # heading 旋轉沒做的話，飛機會往「轉之前」的方向跑 —— 這一項專門抓那個。
        move_test("轉向後前進 linear.x=+0.6", 0.6, 0.0, 4.0)

    # ---------------- C5 逾時交還 ----------------
    rep.section("C5 停止發 cmd_vel 後要交還控制權")
    probe.spin(3.0)      # 不發 cmd_vel
    n = probe.node.count_publishers(f"/{NAMESPACE}/fmu/in/trajectory_setpoint")
    print(f"     trajectory_setpoint 的發布者數：{n}")
    if n > 0:
        rep.fail(f"還有 {n} 個發布者沒收掉。別人（例如 precision_land_node）"
                 "用 count_publishers 檢查「還有沒有人在控制」時會誤判，"
                 "直接拒絕接手 —— 交接會卡死")
    else:
        rep.ok("已停止發布，控制權交還")

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過" + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
