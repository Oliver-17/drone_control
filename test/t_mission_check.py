#!/usr/bin/env python3
# =============================================================================
#  t_mission_check.py — S7 驗證：整趟任務（規劃 → 起飛 → 導航 → 降落）
#
#  用法：
#      python3 src/drone_control/test/t_mission_check.py
#      python3 src/drone_control/test/t_mission_check.py --gui       # 看飛
#      python3 src/drone_control/test/t_mission_check.py --no-land   # 只測導航
#
#  ⚠️ 會自己開關 Gazebo、PX4、Nav2 與降落節點。跑之前確認沒有別的 SITL 在跑。
#
#  六項檢查：
#      C1  前置資料（場地、拓樸圖、AprilTag 位置）
#      C2  route_server 單獨回答得出路線，且節點座標有填（不用 Gazebo）
#      C3  完整堆疊起得來
#      C4  mission_node 跑完，四個階段都走過
#      C5  落點對不對 —— 拿 Gazebo 真值比 AprilTag 的實際位置
#      C6  真的降到地上了
#
#  為什麼 C5 要用 Gazebo 真值：
#      前面每一層比的都是「我們自己算出來的東西」，永遠自洽。
#      S6 的磁偏角錯誤就是這樣躲過 t_bridge_check 和 t_costmap_check 的 ——
#      兩邊都「全過」，但 TF 跟真值差了 2 公尺。
# =============================================================================

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from t_costmap_check import (            # noqa: E402
    Report, Procs, build_env, set_sitl_params, locate_arena,
    NAMESPACE, INSTANCE, MODEL, WORLD, SPAWN, SPAWN_X, SPAWN_Y,
)
from t_nav2_flight_check import gazebo_truth   # noqa: E402

FLIGHT_ALT = 3.0
# 落點容許誤差。T3（降落包自己的實飛測試）實測是 9.8 cm，
# 這裡放寬到 0.5 m：中間多了 Nav2 導航這一段，起始偏移比 T3 大。
MAX_LAND_ERROR = 0.5
# 降落完成後離地高度的上限。x500 的腳架大約 0.18 m 高。
MAX_GROUND_Z = 0.5


def read_tag_pose(world_sdf):
    """從世界檔讀 AprilTag 的實際位置。

    不從 gen_arena.py 的 TAG_POS 讀，也不寫死 (27, 16) ——
    要比對的是「Gazebo 裡那張貼圖到底在哪」，所以來源必須是世界檔本身。
    """
    txt = open(world_sdf, encoding="utf-8").read()
    # <include> 區塊裡同時有 apriltag 的 uri 和 pose
    for blk in txt.split("<include>"):
        if "apriltag" not in blk:
            continue
        m = re.search(r"<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)", blk)
        if m:
            return float(m.group(1)), float(m.group(2))
    return None


def main():
    ap = argparse.ArgumentParser(description="S7：整趟任務驗證")
    ap.add_argument("--gui", action="store_true", help="開 Gazebo 視窗")
    ap.add_argument("--no-land", action="store_true",
                    help="只測到導航結束，不呼叫精準降落")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    ap.add_argument("--goal-node", default="-1")
    args, _ = ap.parse_known_args()

    do_land = not args.no_land
    rep = Report()
    procs = Procs()
    scratch = os.environ.get(
        "CLAUDE_SCRATCH",
        os.path.join("/tmp", f"t_mission_{os.getpid()}"))
    os.makedirs(scratch, exist_ok=True)

    try:
        # ------------------------------------------------------------------
        rep.section("C1 前置資料")
        arena = locate_arena()
        if not arena:
            rep.fail("找不到 drone_nav2_apriltag")
            return 1
        world_sdf = os.path.join(arena, "gz", "worlds", f"{WORLD}.sdf")
        graph_file = os.path.join(arena, "graphs", "nav2_arena.geojson")
        for f in (world_sdf, graph_file):
            if not os.path.isfile(f):
                rep.fail(f"缺檔案：{f}")
                return 1
        tag = read_tag_pose(world_sdf)
        if tag is None:
            rep.fail("世界檔裡找不到 AprilTag 的 pose")
            return 1
        nodes = {}
        for feat in json.load(open(graph_file, encoding="utf-8"))["features"]:
            pr = feat.get("properties", {})
            if "id" in pr and feat["geometry"]["type"] == "Point":
                nodes[pr["id"]] = tuple(feat["geometry"]["coordinates"])
        rep.ok(f"{len(nodes)} 個拓樸節點，AprilTag 在 "
               f"({tag[0]:+.2f}, {tag[1]:+.2f})")

        goal_id = int(args.goal_node)
        if goal_id < 0:
            goal_id = max(nodes)
        gx, gy = nodes[goal_id]
        d_tag = math.hypot(gx - tag[0], gy - tag[1])
        if do_land and d_tag > 2.0:
            rep.warn(f"終點節點 {goal_id} 離 AprilTag {d_tag:.2f} m，"
                     f"可能看不到 tag")
        else:
            rep.ok(f"終點節點 {goal_id} ({gx:+.2f}, {gy:+.2f})，"
                   f"離 AprilTag {d_tag:.2f} m")

        env = build_env(args.px4_dir, arena, headless=not args.gui)

        # ------------------------------------------------------------------
        rep.section("C2 route_server 單獨回答（不用 Gazebo）")
        for c in ("px4", "ruby", "route_server", "planner_server",
                  "controller_server", "bt_navigator", "behavior_server",
                  "map_server", "lifecycle_manager", "px4_tf_node",
                  "cmd_vel_to_px4", "apriltag_node", "precision_land_node",
                  "mission_node"):
            subprocess.run(["pkill", "-x", c], capture_output=True)
        time.sleep(2)

        procs.start("route", ["ros2", "launch", "drone_nav2_apriltag",
                              "fly_nodes.launch.py", "route_only:=true"],
                    env=env, log=f"{scratch}/route.log")
        route_ids = None
        for _ in range(25):
            time.sleep(1)
            r = subprocess.run(
                ["ros2", "action", "send_goal", "/compute_route",
                 "nav2_msgs/action/ComputeRoute",
                 f"{{start_id: {min(nodes)}, goal_id: {goal_id}, "
                 f"use_start: false, use_poses: false}}"],
                env=env, capture_output=True, text=True, timeout=30)
            if "SUCCEEDED" in r.stdout:
                # ⚠️ 一定要抓「nodeid 後面緊接著的那個 position」，不能用
                #    findall(r"x: ...") 掃整份輸出 —— 回覆裡還有一個 path
                #    欄位，裡面有幾百個座標。掃整份的話就算 route.nodes 的
                #    座標全是 0，也會被 path 的座標蓋過去變成假通過。
                pairs = re.findall(
                    r"nodeid: (\d+)\s*\n\s*position:\s*\n"
                    r"\s*x: ([-\d.eE+]+)\s*\n\s*y: ([-\d.eE+]+)",
                    r.stdout)
                route_ids = [int(a) for a, _, _ in pairs]
                node_xy = [(float(x), float(y)) for _, x, y in pairs]
                break
        if not route_ids:
            rep.fail("route_server 25 秒內答不出路線")
            return 1
        rep.ok("路線： " + " -> ".join(str(i) for i in route_ids))
        if not node_xy:
            rep.fail("解析不出 route.nodes 的座標")
            return 1
        # 拿回覆的座標跟 geojson 逐一對帳。只檢查「不是 0」不夠 ——
        # 真正要確認的是「mission_node 會飛到的點，就是圖上那個點」。
        bad = [(i, xy, nodes[i]) for i, xy in zip(route_ids, node_xy)
               if i in nodes
               and math.hypot(xy[0] - nodes[i][0], xy[1] - nodes[i][1]) > 0.01]
        if bad:
            for i, got, want in bad[:3]:
                print(f"     節點 {i}：回覆 ({got[0]:+.2f}, {got[1]:+.2f})"
                      f"  圖上 ({want[0]:+.2f}, {want[1]:+.2f})")
            rep.fail(f"{len(bad)} 個節點的座標跟 geojson 不符 —— 會飛錯地方")
            return 1
        rep.ok(f"{len(node_xy)} 個節點的座標都和 geojson 一致"
               f"（第一個 {node_xy[0][0]:+.2f}, {node_xy[0][1]:+.2f}）")
        # route_server 等下由 mission.launch.py 自己開，這裡先收掉避免撞名
        procs.stop_one("route")
        time.sleep(2)

        # ------------------------------------------------------------------
        rep.section("C3 啟動完整堆疊")
        if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"],
                          capture_output=True).returncode != 0:
            procs.start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888"],
                        env=env, log=f"{scratch}/agent.log")
            time.sleep(2)

        build = os.path.join(args.px4_dir, "build", "px4_sitl_default")
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
        if do_land:
            procs.start("land", ["ros2", "launch", "drone_apriltag_landing",
                                 "precision_land.launch.py",
                                 f"namespace:={NAMESPACE}",
                                 "camera:=camera_down"],
                        env=env, log=f"{scratch}/land.log")
        time.sleep(12)
        rep.ok("px4_bridge / 感測器 / Nav2" +
               ("／精準降落" if do_land else "") + " 都已啟動")

        # ------------------------------------------------------------------
        rep.section("C4 跑 mission_node")
        mission_log = f"{scratch}/mission.log"
        t0 = time.time()
        with open(mission_log, "w") as lf:
            mp = subprocess.Popen(
                ["ros2", "launch", "drone_control", "mission.launch.py",
                 f"namespace:={NAMESPACE}", f"flight_altitude:={FLIGHT_ALT}",
                 f"goal_node:={args.goal_node}",
                 "land:=" + ("true" if do_land else "false")],
                env=env, stdout=lf, stderr=subprocess.STDOUT)
            # 上限：規劃 30 + 起飛 60 + 導航 8 段 x 120 + 降落 180，再加餘裕
            limit = 1200
            while mp.poll() is None and time.time() - t0 < limit:
                time.sleep(2)
            if mp.poll() is None:
                rep.fail(f"mission 超過 {limit} 秒還沒結束，強制終止")
                mp.kill()
                mp.wait(timeout=10)

        elapsed = time.time() - t0
        log = open(mission_log, encoding="utf-8", errors="replace").read()
        print(f"     耗時 {elapsed:.0f} 秒，離開碼 {mp.returncode}")

        stages = ["ROUTE -> TAKEOFF", "TAKEOFF -> NAVIGATE"]
        if do_land:
            stages.append("NAVIGATE -> LAND")
            stages.append("LAND -> DONE")
        else:
            stages.append("NAVIGATE -> DONE")
        missing = [s for s in stages if s not in log]
        if missing:
            rep.fail("沒走完的階段轉換： " + "、".join(missing))
            for line in log.splitlines()[-25:]:
                print("     " + line)
            return 1
        rep.ok("四個階段都走過： " + " / ".join(stages))

        arrived = re.findall(r"✓ 到達節點 (\d+)", log)
        rep.ok(f"到達 {len(arrived)} 個節點： " + " ".join(arrived))
        recov = re.findall(r"已觸發 (\d+) 次復原行為", log)
        if recov:
            rep.warn(f"途中觸發過復原行為（最多 {max(int(r) for r in recov)} 次）"
                     f"—— 代表卡住過，值得看一下路徑")

        if do_land:
            m = re.search(
                r"殘餘誤差 x ([-+\d.]+) m、y ([-+\d.]+) m、yaw ([-+\d.]+)°", log)
            if m:
                rep.ok(f"降落節點自己報的殘餘誤差： "
                       f"x {m.group(1)} m、y {m.group(2)} m、"
                       f"yaw {m.group(3)}°")
            else:
                rep.warn("log 裡找不到降落誤差那一行")

        # ------------------------------------------------------------------
        rep.section("C5 落點（對 Gazebo 真值）")
        truth = gazebo_truth(env)
        if truth is None:
            rep.fail("問不到 Gazebo 真值")
            return 1
        tx, ty, tz, tyaw = truth
        print(f"     真值            : ({tx:+.2f}, {ty:+.2f}, {tz:+.2f})")
        if do_land:
            print(f"     AprilTag 實際位置: ({tag[0]:+.2f}, {tag[1]:+.2f})")
            err = math.hypot(tx - tag[0], ty - tag[1])
            print(f"     落點誤差        : {err:.3f} m")
            if err > MAX_LAND_ERROR:
                rep.fail(f"落點離 AprilTag {err:.3f} m，超過 {MAX_LAND_ERROR} m")
            else:
                rep.ok(f"落在 AprilTag 上（差 {err:.3f} m）")
        else:
            print(f"     終點節點        : ({gx:+.2f}, {gy:+.2f})")
            err = math.hypot(tx - gx, ty - gy)
            print(f"     到點誤差        : {err:.3f} m")
            # 沒降落時只要求到 Nav2 的容差附近（xy_goal_tolerance 0.4）
            if err > 1.0:
                rep.fail(f"離終點節點 {err:.3f} m，超過 1.0 m")
            else:
                rep.ok(f"停在終點節點上空（差 {err:.3f} m）")

        # ------------------------------------------------------------------
        rep.section("C6 是否真的落地")
        if do_land:
            if tz > MAX_GROUND_Z:
                rep.fail(f"真值高度 {tz:.2f} m，還在空中（門檻 {MAX_GROUND_Z} m）")
            else:
                rep.ok(f"已在地面（高度 {tz:.2f} m）")
        else:
            if abs(tz - FLIGHT_ALT) > 1.0:
                rep.warn(f"高度 {tz:.2f} m，離設定的 {FLIGHT_ALT} m 有點遠")
            else:
                rep.ok(f"維持在巡航高度（{tz:.2f} m）")

    finally:
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
