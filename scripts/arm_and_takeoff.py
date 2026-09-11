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
    def __init__(self, ns, altitude, target_system):
        super().__init__("arm_and_takeoff")
        self.altitude = altitude
        self.ts = target_system
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
        end = time.time() + 60
        stable = 0
        while time.time() < end:
            self.hold(0.5)
            if abs(-self.pos.z - self.altitude) < 0.2 and abs(self.pos.vz) < 0.2:
                stable += 1
                if stable >= 4:
                    break
            else:
                stable = 0
            print(f"  高度 {-self.pos.z:5.2f} / {self.altitude:.1f} m", end="\r")
        print()
        if stable < 4:
            print(f"✗ 60 秒內沒穩定在 {self.altitude} m"
                  f"（目前 {-self.pos.z:.2f} m）")
            print("  確認 cmd_vel_to_px4_node 的 flight_altitude 和這裡一致")
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
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ArmAndTakeoff(args.ns, args.altitude, args.target_system)
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
