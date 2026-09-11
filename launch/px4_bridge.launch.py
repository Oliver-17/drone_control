"""
PX4 <-> ROS 雙向座標翻譯（S1 + S2）。

    px4_tf_node          PX4 -> ROS   發布 TF: map -> odom -> base_link
    cmd_vel_to_px4_node  ROS  -> PX4  收 <ns>/cmd_vel，轉成 TrajectorySetpoint

有了這兩支，PX4 對 ROS 來說就像一台普通的 ROS 機器人 ——
Nav2 根本不需要知道底下是 PX4。

用法:
    # 前提：SITL 已經在跑，且 MicroXRCEAgent 已開
    ros2 launch drone_control px4_bridge.launch.py
    ros2 launch drone_control px4_bridge.launch.py namespace:=MAV2
    ros2 launch drone_control px4_bridge.launch.py flight_altitude:=4.0

手動測試:
    ros2 topic pub /MAV1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
    （飛機應該往機頭方向前進，高度不變）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_control"


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    alt = LaunchConfiguration("flight_altitude").perform(context)
    cfg = os.path.join(get_package_share_directory(PKG), "config", "px4_bridge.yaml")

    # 兩支都放在同一個 namespace 底下：
    # cmd_vel_to_px4_node 訂的是相對名稱 "cmd_vel"，放進 namespace 之後
    # 就變成 /MAV1/cmd_vel —— 多機時三台各自吃自己的指令，不會互相干擾。
    common = {"px4_namespace": ns}
    origin = LaunchConfiguration("odom_origin").perform(context)
    tf_extra = dict(common)
    if origin:
        vals = [float(v) for v in origin.split(",")]
        tf_extra["odom_origin_in_map"] = (vals + [0.0, 0.0, 0.0])[:3]

    return [
        Node(
            package=PKG, executable="px4_tf_node", name="px4_tf_node",
            namespace=ns, parameters=[cfg, tf_extra],
            output="screen", emulate_tty=True,
        ),
        Node(
            package=PKG, executable="cmd_vel_to_px4_node",
            name="cmd_vel_to_px4_node",
            namespace=ns,
            parameters=[cfg, common, {"flight_altitude": float(alt)}],
            output="screen", emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 namespace，多機時改這個"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0",
                              description="定高飛行的高度（公尺）"),
        DeclareLaunchArgument("odom_origin", default_value="",
                              description="飛機開機位置在 map 的哪裡，格式 x,y,z（ENU）。"
                                          "留空就用參數檔的值"),
        OpaqueFunction(function=_setup),
    ])
