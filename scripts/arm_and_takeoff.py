#!/usr/bin/env python3
# =============================================================================
#  arm_and_takeoff.py — 解鎖 + 起飛到定高，然後把控制權交出去
#
#  用法：
#      ros2 run drone_control arm_and_takeoff.py
#      ros2 run drone_control arm_and_takeoff.py --altitude 4.0 --ns MAV2
#
#  為什麼需要它：
#      cmd_vel_to_px4_node 是「有人送 cmd_vel 才接管」的設計 ——
#      在 Nav2 送出第一個指令之前，飛機根本還沒解鎖、還在地上。
#      這支就是那個「先把飛機弄到空中」的動作。
#
#  它怎麼運作：
#      發零速度的 cmd_vel（cmd_vel_to_px4_node 收到就會開始發 setpoint）
#      → 送解鎖與切 offboard 的指令
#      → 等爬到目標高度
#      → 停止發布並退出，把 cmd_vel 讓給 Nav2
#
#      退出之後 PX4 會在約 0.5 秒內掉進 HOLD（原地懸停）——
#      這是安全的。Nav2 一開始送 cmd_vel，cmd_vel_to_px4_node 會自己
#      把模式切回 offboard。
#
#  ⚠️ 它「不會」持續發零速度。持續發的話會跟 Nav2 的指令交錯，
#     速度變成一半、而且會抖 —— 兩個發布者搶同一個 topic 的典型症狀。
# =============================================================================

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus


class ArmAndTakeoff(Node):
    def __init__(self, ns, altitude, target_system,
                 tolerance=0.08, vz_tolerance=0.05, stable_count=6, timeout=90.0):
        super().__init__("arm_and_takeoff")
        self.altitude = altitude
        self.ts = target_system
        # 「到底算不算到了」的判定門檻。做成參數是因為不同高度該用不同的值 ——
        # 室內飛 0.5 m 和室外飛 3 m，0.2 m 的誤差意義完全不同（40% vs 7%）。
        self.tolerance = tolerance
        self.vz_tolerance = vz_tolerance
        self.stable_count = stable_count
        # 逾時要留足夠裕度：門檻收緊之後收斂變慢，而逾時代表「腳本退出但飛機還在空中」
        self.timeout = timeout
        self.pos = None
        self.status = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
            lambda m: setattr(self, "pos", m), qos)
        self.create_subscription(
            VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "status", m), qos)
        self.cmd_pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.vc_pub = self.create_publisher(
            VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)

    def hold(self, seconds, rate_hz=20.0):
        """發零速度的 cmd_vel 並跑 ROS 迴圈。

        ⚠️ 一定要限速。spin_once 幾乎立刻返回，直接在迴圈裡發等於用數千 Hz 灌，
        PX4 的佇列會塞爆 —— 症狀是「解鎖了也進 offboard 了，就是不動」。
        """
        m = Twist()
        end = time.time() + seconds
        period, nxt = 1.0 / rate_hz, 0.0
        while time.time() < end:
            t = time.time()
            if t >= nxt:
                self.cmd_pub.publish(m)
                nxt = t + period
            rclpy.spin_once(self, timeout_sec=0.02)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = self.ts
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True    # 少了這個 PX4 會當成內部指令直接拒絕
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vc_pub.publish(m)

    def run(self):
        print("等 PX4 的位置估計…")
        end = time.time() + 90
        while time.time() < end and (
                self.pos is None or not self.pos.xy_valid or not self.pos.z_valid):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pos is None:
            print("✗ 收不到 vehicle_local_position —— "
                  "確認 SITL 和 MicroXRCEAgent 都在跑")
            return 1
        print(f"✓ 目前高度 {-self.pos.z:.2f} m")

        # PX4 要求「先有 setpoint 串流，才准切 offboard」。
        # 順序反過來的話切模式指令會被拒絕，而且訊息很不明顯。
        self.hold(2.0)
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.hold(0.5)
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.hold(1.0)

        for _ in range(10):
            s = self.status
            armed = s is not None and s.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offb = s is not None and s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if armed and offb:
                break
            if not offb:
                self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if not armed:
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.hold(1.0)
        else:
            print(f"✗ 解鎖或切 offboard 失敗"
                  f"（arming={self.status.arming_state if self.status else '?'}"
                  f" nav={self.status.nav_state if self.status else '?'}）")
            print("  常見原因：預檢沒過。看 PX4 的 out.log")
            return 1
        print("✓ 已解鎖並進入 offboard，開始爬升…")

        # cmd_vel_to_px4_node 收到零速度就會把高度鎖在它的 flight_altitude，
        # 所以「什麼都不做」就會自己爬上去。
        # ⚠️ 門檻不能太寬。高度是 P 控制器（kp_z 預設 0.6），速度指令 = 0.6 × 誤差，
        #    所以「越接近目標速度越慢」—— 誤差 0.2 m 時速度才 0.12 m/s。
        #    舊版用 0.2 m / 0.2 m/s，兩個條件在飛機**還在爬的半路**就同時成立，
        #    連續判定幾次都沒用（它根本不會再跳出門檻）。
        #    2026-10-01 實機：目標 0.5 m，飛機爬到 0.31 m 就被判定「已懸停」，
        #    離地才兩秒腳本就退出，之後沒人控制 → 飄移 + 失效保護降落。
        #    現在的預設（0.08 m / 0.05 m/s）用同一條公式驗算：
        #    誤差 0.08 → 速度指令 0.048 m/s，剛好在門檻內，所以兩個條件
        #    會在「真的快停住」時才一起成立。
        end = time.time() + self.timeout
        stable = 0
        while time.time() < end:
            self.hold(0.5)
            err = abs(-self.pos.z - self.altitude)
            if err < self.tolerance and abs(self.pos.vz) < self.vz_tolerance:
                stable += 1
                if stable >= self.stable_count:
                    break
            else:
                stable = 0
            print(f"  高度 {-self.pos.z:5.2f} / {self.altitude:.2f} m"
                  f"   誤差 {err:4.2f}   vz {self.pos.vz:+5.2f} m/s"
                  f"   穩定 {stable}/{self.stable_count}  ", end="\r")
        print()
        if stable < self.stable_count:
            print(f"✗ {self.timeout:.0f} 秒內沒穩定在 {self.altitude} m"
                  f"（目前 {-self.pos.z:.2f} m、誤差門檻 {self.tolerance} m）")
            print("  確認 cmd_vel_to_px4_node 的 flight_altitude 和這裡一致")
            print("  如果高度卡在某個值收斂不到門檻，就放寬 --tolerance")
            print("  ⚠️ 飛機現在還在空中，而這支已經不再送 cmd_vel —— 用遙控器接手")
            return 1

        print(f"✓ 已懸停在 {-self.pos.z:.2f} m")
        print()
        print("控制權交還。接下來：")
        print("  在 RViz 按「2D Goal Pose」點一個位置，Nav2 就會接手飛過去。")
        print("  （這支停止發 cmd_vel 之後 PX4 會掉進 HOLD 原地懸停，")
        print("    Nav2 一送指令 cmd_vel_to_px4_node 會自己切回 offboard。）")
        return 0


def main():
    ap = argparse.ArgumentParser(description="解鎖 + 起飛到定高，然後交出控制權")
    ap.add_argument("--ns", default="MAV1")
    ap.add_argument("--altitude", type=float, default=3.0,
                    help="要和 cmd_vel_to_px4_node 的 flight_altitude 一致")
    ap.add_argument("--target-system", type=int, default=1,
                    help="多機時是 instance+1")
    ap.add_argument("--tolerance", type=float, default=0.08,
                    help="高度誤差在這個值以內才算到位（公尺）")
    ap.add_argument("--vz-tolerance", type=float, default=0.05,
                    help="垂直速度在這個值以內才算穩定（m/s）")
    ap.add_argument("--stable-count", type=int, default=6,
                    help="要連續幾次都通過才算數（0.5 秒一次，6 次 = 3 秒）")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="等多久就放棄（秒）。逾時的話飛機還在空中，要用遙控器接手")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ArmAndTakeoff(args.ns, args.altitude, args.target_system,
                         args.tolerance, args.vz_tolerance, args.stable_count,
                         args.timeout)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        print("\n(中斷)")
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
