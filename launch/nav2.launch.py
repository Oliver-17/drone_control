"""
Nav2 導航堆疊，分層啟動（S4 / S5 / S6）。

    level:=planner   （預設）map_server + planner_server + goal_tool
                     只會規劃、畫路徑，飛機不會動          ← S4/S5
    level:=full      再加 controller_server + bt_navigator + behavior_server
                     RViz 點目標，飛機真的飛過去           ← S6

⚠️ 為什麼 costmap 要由這些伺服器帶起來，不能用 nav2_costmap_2d 單獨跑：
    Costmap2DROS 的名稱與命名空間寫死在 C++
    （costmap_2d_ros.hpp:83-87「the node will be placed in a namespace
    equal to the node's name」），launch 的 name= / namespace= 都蓋不掉，
    FQN 永遠是 /costmap/costmap —— 跑兩個會撞名，兩個都廢掉。
    planner_server 帶 global_costmap、controller_server 帶 local_costmap。

前提：
    1. SITL 在跑（MicroXRCEAgent 也要開）
    2. px4_bridge.launch.py 在跑（TF 和 /odom 靠它）
    3. cameras.launch.py 在跑（光達、/clock、感測器靜態 TF）

用法：
    ros2 launch drone_control nav2.launch.py rviz:=true                # S5
    ros2 launch drone_control nav2.launch.py level:=full rviz:=true    # S6

S6 要飛之前，飛機必須先解鎖並爬到高度：
    ros2 run drone_control arm_and_takeoff.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARENA = "drone_nav2_apriltag"


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    params = LaunchConfiguration("params_file").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)

    show_rviz = LaunchConfiguration("rviz").perform(context).lower() \
        in ("true", "1", "yes")
    level = LaunchConfiguration("level").perform(context).lower()
    full = (level == "full")
    arena_share = get_package_share_directory(ARENA)
    if not params:
        params = os.path.join(arena_share, "config", "nav2_params.yaml")
    if not map_yaml:
        map_yaml = os.path.join(arena_share, "maps", "nav2_arena.yaml")

    # 光達的 topic 帶 namespace，參數檔裡寫的是預設的 MAV1。
    # 鍵名要用「點」串起巢狀層級，而且前面不含節點名。
    scan_override = {
        "global_costmap.global_costmap.obstacle_layer.scan.topic": f"/{ns}/scan",
        "local_costmap.local_costmap.obstacle_layer.scan.topic": f"/{ns}/scan",
        "bt_navigator.odom_topic": f"/{ns}/odom",
    }

    extra = []
    if show_rviz:
        # 設定檔裡已經開好「全域 costmap」「光達」「無人機位置」三個 display，
        # Fixed Frame 設成 map、視角是俯視整個場地。
        # use_sim_time 要跟著開，否則 RViz 會用系統時間去比對帶 sim time 的資料，
        # 畫面會一直閃或乾脆不顯示。
        extra.append(Node(
            package="rviz2", executable="rviz2", name="rviz2_costmap",
            arguments=["-d", os.path.join(arena_share, "rviz",
                                          "nav2_costmap.rviz")],
            parameters=[{"use_sim_time": True}],
            output="log",
        ))

    want_goal_tool = LaunchConfiguration("goal_tool").perform(context).lower() \
        in ("true", "1", "yes")
    # full 模式下 bt_navigator 會接收 /goal_pose 並真的執行導航，
    # 這時再開 goal_tool 會變成「兩個人同時規劃」，只是浪費 CPU。
    if want_goal_tool and not full:
        # 把 RViz 的「2D Goal Pose」翻譯成呼叫 planner_server。
        # 那個按鈕發的是 /goal_pose，平常由 bt_navigator 接收 ——
        # 但 bt_navigator 要 S6 才會啟動，在只有 planner 的階段點下去沒反應。
        extra.append(Node(
            package="drone_control", executable="goal_to_planner_node",
            name="goal_to_planner_node",
            parameters=[{"use_sim_time": True}],
            output="screen", emulate_tty=True,
        ))

    return extra + [
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[params, {"yaml_filename": map_yaml}],
            output="screen", emulate_tty=True,
        ),
        # ⚠️ costmap 必須由 planner_server 帶起來，不能用 nav2_costmap_2d 單獨跑。
        #    Costmap2DROS 的名稱與命名空間寫死在 C++ 裡（costmap_2d_ros.hpp:83-87
        #    「the node will be placed in a namespace equal to the node's name」），
        #    launch 的 name= / namespace= 都蓋不掉，FQN 永遠是 /costmap/costmap ——
        #    同時跑全域和區域兩個會撞名，兩個都廢掉。
        #
        #    planner_server 會建立名為 global_costmap 的子節點，
        #    讀的就是參數檔裡 global_costmap: global_costmap: 那一段。
        #    S4 只是借它把 costmap 帶起來，還不會送目標給它。
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server",
            parameters=[params, scan_override],
            output="screen", emulate_tty=True,
        ),
    ] + ([
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server",
            parameters=[params, scan_override],
            # controller_server 發的是 /cmd_vel，而 cmd_vel_to_px4_node
            # 訂的是 /<ns>/cmd_vel —— 不 remap 的話飛機收不到任何指令，
            # 而且兩邊都不會報錯，只會「規劃有了但飛機不動」。
            remappings=[("cmd_vel", f"/{ns}/cmd_vel")],
            output="screen", emulate_tty=True,
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", parameters=[params],
            remappings=[("cmd_vel", f"/{ns}/cmd_vel")],
            output="screen", emulate_tty=True,
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", parameters=[params, scan_override],
            output="screen", emulate_tty=True,
        ),
    ] if full else []) + [
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_costmap",
            parameters=[{
                "autostart": True,
                "node_names": (["map_server", "planner_server"] +
                               (["controller_server", "behavior_server",
                                 "bt_navigator"] if full else [])),
                # 沒有 /clock 發布者，設 true 會讓計時器永遠不觸發
                "use_sim_time": False,
            }],
            output="screen", emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 namespace，決定光達 topic"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="留空就用 drone_nav2_apriltag 的 nav2_params.yaml"),
        DeclareLaunchArgument("map", default_value="",
                              description="留空就用 drone_nav2_apriltag 的 nav2_arena.yaml"),
        DeclareLaunchArgument("level", default_value="planner",
                              description="planner=只規劃（S4/S5）；"
                                          "full=加上控制器，飛機會動（S6）"),
        DeclareLaunchArgument("goal_tool", default_value="true",
                              description="把 RViz 的 2D Goal Pose 接到 planner。"
                                          "S6 之後 bt_navigator 會接管，要設 false"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="true 會順便開 RViz，設定檔已經配好"),
        OpaqueFunction(function=_setup),
    ])
