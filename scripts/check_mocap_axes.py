#!/usr/bin/env python3
# =============================================================================
#  check_mocap_axes.py — 動捕的「位置」和「姿態」有沒有講同一套話
#
#  用途：實機室內飛行前的決定性檢查。**不用起飛、不用解鎖、不用裝螺旋槳**，
#       用手抱著飛機走兩趟就有答案。
#
#  用法：
#      export ROS_DOMAIN_ID=42        # 要和飛控的 UXRCE_DDS_DOM_ID 一致
#      ros2 run drone_control check_mocap_axes.py
#      ros2 run drone_control check_mocap_axes.py --ns MAV2
#
#  這支程式「只訂閱、不發布」，不會對飛機下任何指令。
#
#  ---------------------------------------------------------------------------
#  為什麼要測這個
#  ---------------------------------------------------------------------------
#  室內沒有 GPS，位置來自動捕（OptiTrack）。動捕的資料要從 ENU 轉成 PX4 的
#  NED 才能餵進 EKF2，而這個轉換很容易寫錯：
#
#      正確： x_ned = y_enu,  y_ned = x_enu,  z_ned = -z_enu   （x/y 互換）
#      常見錯誤： x_ned = x_enu,  y_ned = -y_enu               （把機體的
#                 FLU→FRD 那組 (x,-y,-z) 誤用到世界座標系上）
#
#  ⚠️ 關鍵：轉錯之後得到的仍然是一個「合法的右手座標系」，只是繞垂直軸轉了
#     90 度。室內沒有真北，座標系轉幾度本來就無所謂 ——
#     **真正會出事的是「位置轉了、姿態沒轉」**，兩者差 90 度。
#     那會讓位置控制器把飛機往垂直於目標的方向推，而且越推越遠（發散）。
#
#  所以這支程式不檢查「北是不是真的北」（那沒有意義），
#  只檢查「位置」和「姿態」是不是同一個座標系。
#
#  ---------------------------------------------------------------------------
#  原理
#  ---------------------------------------------------------------------------
#  VehicleLocalPosition 裡的 heading 定義是「相對 NED 的偏航角」
#  （px4_msgs/msg/VehicleLocalPosition.msg:42）。所以：
#
#      heading = 0  ⇒  機頭指向 +x 方向
#      ⇒ 沿機頭方向走，x 應該增加、y 幾乎不變
#
#  如果位置被轉了 90 度而姿態沒轉，沿機頭方向走會變成 y 在動。
#
#  這是**定性**測試：看的是「哪個軸在動」，不是「動了多少」，
#  所以你不需要走得很準，走個一兩公尺就夠。
# =============================================================================

import argparse
import math
import os
import select
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from px4_msgs.msg import VehicleLocalPosition

# 判定門檻
MOVE_MIN = 0.5      # 移動超過這麼多公尺才開始下判定（太小的話雜訊會主導）
DOMINANT = 3.0      # 一軸要比另一軸大這麼多倍，才算「幾乎完全在這個軸上」
HEADING_OK = 10.0   # 記基準點時 heading 要在這個度數內，判定才有意義

RENDER_LINES = 11   # 畫面固定佔幾行，用來把游標移回去重畫


class MocapAxes(Node):
    def __init__(self, ns):
        super().__init__("check_mocap_axes")

        # ⚠️ 一定要 BEST_EFFORT。PX4 的 uXRCE-DDS client 是用 BEST_EFFORT
        #    發布的，rclpy 預設是 RELIABLE —— 兩者不相容，會「安靜地」
        #    收不到任何資料。（ros2 topic echo 看得到，因為它會自動匹配 QoS，
        #    所以更容易誤判成「程式寫錯」。）
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)

        self.topic = f"/{ns}/fmu/out/vehicle_local_position_v1"
        self.create_subscription(VehicleLocalPosition, self.topic,
                                 self._on_pos, qos)
        self.pos = None
        self.count = 0
        self.ref = None          # (x, y, z)
        self.ref_heading = None  # 度

    def _on_pos(self, m):
        self.pos = m
        self.count += 1

    def mark(self):
        if self.pos is None:
            return False
        self.ref = (self.pos.x, self.pos.y, self.pos.z)
        self.ref_heading = math.degrees(self.pos.heading)
        return True

    def clear(self):
        self.ref = None
        self.ref_heading = None


def verdict(dx, dy, ref_heading):
    """把觀察到的位移換成結論。

    刻意「不」假設使用者往哪個方向走 —— 程式不知道他剛才走的是前面還是
    右邊，硬猜的話會給出自信但錯誤的答案。所以這裡只說「位移落在哪個軸」，
    再把兩種走法的意義都列出來，由使用者對照。
    """
    a, b = abs(dx), abs(dy)
    if max(a, b) < MOVE_MIN:
        return None, f"還沒走夠遠（至少 {MOVE_MIN:.1f} m）"

    if ref_heading is not None and abs(ref_heading) > HEADING_OK:
        return "warn", (f"記基準時 heading 是 {ref_heading:+.1f}°，"
                        f"不在 ±{HEADING_OK:.0f}° 內 —— 判定不準，請重轉正再測")

    if a > DOMINANT * b:
        return "x", "位移幾乎完全在 x 軸"
    if b > DOMINANT * a:
        return "y", "位移幾乎完全在 y 軸"
    return "mix", "x 和 y 都在動 —— 可能 heading 沒對準 0，或走的方向是斜的"


def render(node, args):
    p = node.pos
    out = []
    out.append("═" * 64)
    out.append(f" 動捕座標軸自洽檢查    topic: {node.topic}")
    out.append("═" * 64)

    if p is None:
        out.append("")
        out.append("  ⏳ 還沒收到資料。檢查這幾件事：")
        out.append(f"       ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '(沒設)')}"
                   f"  ← 要和飛控的 UXRCE_DDS_DOM_ID 一致")
        out.append("       MicroXRCEAgent 有在跑嗎（Pi 4 上，走序列埠）")
        out.append("       ros2 topic list | grep vehicle_local_position")
        out.append("")
        out.append("")
        out.append("")
        return out

    alt = -p.z
    hdg = math.degrees(p.heading)

    # ⚠️ 判斷「動捕有沒有進 EKF2」要以 xy_valid 為準，不能只看 eph。
    #    2026-09-16 實測踩到：xy_valid=false 但 eph 顯示 0.012（公分級），
    #    位置卻漂到 (-165, +307) —— 那個 eph 是估計失效前留下的殘值，
    #    完全沒有意義。只看 eph 會得到「動捕正常」的錯誤結論。
    ok = p.xy_valid and p.z_valid
    if not ok:
        eph_note = "⚠️  位置無效時 eph 沒有意義，看下面那行"
    elif p.eph < 0.10:
        eph_note = "✅ 公分級，動捕有在融合"
    elif p.eph < 0.50:
        eph_note = "⚠️  偏大，確認動捕沒有斷斷續續"
    else:
        eph_note = "❌ 太大，EKF 可能在靠慣性推算（動捕沒進去？）"

    # ⚠️ 位置和速度是兩個獨立的旗標。2026-09-16 實測遇到
    #    「xy_valid=false 但相對位移很準」——那代表 EKF2 在融合動捕的
    #    「速度」而沒融合「位置」：相對移動對，絕對位置從開機漂到哪算哪。
    #    只看 xy_valid 會誤判成「完全沒在融合」，所以兩個都要顯示。
    vok = getattr(p, "v_xy_valid", False)
    valid = (f"位置 {'✅' if ok else '❌'}   速度 {'✅' if vok else '❌'}")
    if not ok and vok:
        valid += ("\n            ⚠️ 速度有效但位置無效 = 只融合了速度，沒融合位置。\n"
                  "               相對位移可信（座標軸判定仍有效），\n"
                  "               但絕對位置不可用 —— Nav2 這樣不能飛。\n"
                  "               查 EKF2_EV_CTRL 的 bit0（水平位置）有沒有開。")
    elif not ok:
        valid += ("\n            ❌ 位置與速度都無效 —— EKF2 沒在用動捕。\n"
                  "               查 estimator_status_flags 的 cs_ev_pos，\n"
                  "               以及 QGC 的 EKF2_EV_CTRL（預設 0 = 全關）")

    out.append("")
    hint = "  ← 可以了，記基準點" if abs(hdg) <= HEADING_OK else "  ← 請轉到接近 0"
    out.append(f"  heading  {hdg:+7.1f}°{hint}")
    out.append(f"  x(北) {p.x:+7.2f}   y(東) {p.y:+7.2f}   "
               f"z(下) {p.z:+7.2f}   離地 {alt:+.2f} m")
    out.append(f"  eph {p.eph:.3f} m   {eph_note}")
    out.append(f"  有效性   {valid}")
    out.append("")

    if node.ref is None:
        out.append("  [Enter] 記下基準點，然後抱著飛機走")
        out.append("")
        out.append("")
    else:
        dx = p.x - node.ref[0]
        dy = p.y - node.ref[1]
        out.append(f"  Δx {dx:+7.2f}   Δy {dy:+7.2f}   "
                   f"（基準時 heading {node.ref_heading:+.1f}°）")
        kind, why = verdict(dx, dy, node.ref_heading)
        if kind is None or kind == "warn":
            out.append(f"  {why}")
            out.append("")
        elif kind == "mix":
            out.append(f"  ⚠️  {why}")
            out.append("")
        elif kind == "x":
            out.append(f"  {why}：")
            out.append("     往機頭方向走 → ✅ 自洽　|　往右走 → ❌ 位置轉了 90°")
        else:
            out.append(f"  {why}：")
            out.append("     往機頭方向走 → ❌ 位置轉了 90°　|　往右走 → ✅ 自洽")
        out.append("  [Enter] 重新記基準點")
    return out


def main():
    ap = argparse.ArgumentParser(
        description="檢查動捕的位置與姿態是否在同一個座標系（唯讀，不碰飛機）")
    ap.add_argument("--ns", default="MAV1", help="PX4 namespace")
    args, _ = ap.parse_known_args()

    print("\n抱著飛機水平拿好，轉到 heading ≈ 0，按 Enter 記基準點，"
          "然後往機頭方向走 1~2 公尺。\nCtrl+C 結束。\n")

    rclpy.init()
    node = MocapAxes(args.ns)
    tty = sys.stdout.isatty()
    printed = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # 非阻塞讀 stdin：不能直接 input()，那會卡住 spin，
            # 訂閱就停止收資料、畫面凍住。
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                if node.ref is None:
                    if not node.mark():
                        pass
                else:
                    node.clear()

            lines = render(node, args)
            if tty:
                if printed:
                    sys.stdout.write(f"\033[{printed}A")   # 游標移回開頭
                for l in lines:
                    sys.stdout.write("\033[2K" + l + "\n")  # 清行再寫
                printed = len(lines)
                sys.stdout.flush()
            else:
                # 不是終端機（重導到檔案）就別用 ANSI，直接印
                for l in lines:
                    print(l)
    except KeyboardInterrupt:
        print("\n(結束)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
