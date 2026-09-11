# =============================================================================
#  mission.launch.py — S7：整趟任務（規劃 → 起飛 → 導航 → 降落）
#
#  用法：
#      ros2 launch drone_control mission.launch.py
#      ros2 launch drone_control mission.launch.py goal_node:=7 land:=false
#      ros2 launch drone_control mission.launch.py namespace:=MAV2 target_system:=2
#
#  前置條件（這個 launch 檔不會幫你開）：
#      終端 1： DRONES=1 ~/ros2_ws/src/drone_nav2_apriltag/scripts/start_arena_sitl.sh
#      終端 2： MicroXRCEAgent udp4 -p 8888
#      終端 3： ros2 launch drone_control px4_bridge.launch.py \
#                   namespace:=MAV1 odom_origin:=-2,0,0 flight_altitude:=3.0
#      終端 4： ros2 launch drone_nav2_apriltag cameras.launch.py \
#                   namespace:=MAV1 view:=false lidar:=true
#      終端 5： ros2 launch drone_control nav2.launch.py namespace:=MAV1 level:=full
#      （降落要用的話還要： ros2 launch drone_apriltag_landing
#        precision_land.launch.py namespace:=MAV1）
#
#  這個 launch 檔起兩樣東西：
#      1. route_server + lifecycle_manager   ← include 地圖套件的設定，不複製
#      2. mission_node                        ← 任務編排
#
#  為什麼 route_server 是 include 來的而不是這裡自己宣告：
#      route_server 的 edge_cost_functions 必須明寫（預設不含 PenaltyScorer，
#      地圖裡的 penalty 會完全失效且沒有警告）。那組設定只該有一份，
#      所以這裡 include drone_nav2_apriltag 的 fly_nodes.launch.py 並帶
#      route_only:=true —— 那個參數會關掉 fly_nodes 本體，只留 route_server。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARENA = "drone_nav2_apriltag"


def generate_launch_description():
    arena_share = get_package_share_directory(ARENA)
    default_graph = os.path.join(arena_share, "graphs", "nav2_arena.geojson")

    args = [
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 namespace，多機時改這個"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="多機時是 instance+1"),
        DeclareLaunchArgument("graph", default_value=default_graph,
                              description="拓樸圖檔（route_server 和 mission_node 讀同一份）"),
        DeclareLaunchArgument("start_node", default_value="-1",
                              description="起點節點 id，-1 表示用最小的"),
        DeclareLaunchArgument("goal_node", default_value="-1",
                              description="終點節點 id，-1 表示用最大的（降落點）"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0",
                              description="要和 cmd_vel_to_px4_node 的設定一致"),
        DeclareLaunchArgument("land", default_value="true",
                              description="false 飛到終點就停，不呼叫精準降落"),
        DeclareLaunchArgument("nav_timeout", default_value="120.0",
                              description="單段導航的上限（秒）"),
        DeclareLaunchArgument("land_timeout", default_value="180.0",
                              description="精準降落的上限（秒）"),
    ]

    graph = LaunchConfiguration("graph")

    # route_only:=true → 只有 route_server + lifecycle_manager，沒有 fly_nodes
    route_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arena_share, "launch", "fly_nodes.launch.py")),
        launch_arguments={"route_only": "true", "graph": graph}.items(),
    )

    mission = Node(
        package="drone_control",
        executable="mission_node.py",
        name="mission_node",
        output="screen",
        parameters=[{
            "namespace": LaunchConfiguration("namespace"),
            "target_system": LaunchConfiguration("target_system"),
            "graph_file": graph,
            "start_node": LaunchConfiguration("start_node"),
            "goal_node": LaunchConfiguration("goal_node"),
            "flight_altitude": LaunchConfiguration("flight_altitude"),
            "land": LaunchConfiguration("land"),
            "nav_timeout": LaunchConfiguration("nav_timeout"),
            "land_timeout": LaunchConfiguration("land_timeout"),
        }],
    )

    # 任務結束＝mission_node 退出。route_server 是常駐的，不主動收掉
    # 整個 launch 會一直掛著等你 Ctrl+C。
    # （同樣的手法見 drone_nav2_apriltag 的 fly_nodes.launch.py）
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(target_action=mission,
                      on_exit=[Shutdown(reason="mission_node 已結束，關閉整組")]))

    return LaunchDescription(args + [route_server, mission, shutdown_when_done])
