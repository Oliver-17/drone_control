#!/usr/bin/env python3
"""
single_drone.launch.py — 單機 Offboard 起飛（MAV1）

【重要：機名與 topic 前綴的關係】

  vehicle_name  = "MAV1"   → 只是給人看的名字，出現在 log 裡，不影響任何 topic
  px4_namespace = "/MAV1"  → 真正的 topic 前綴，會變成 /MAV1/fmu/in/...

  px4_namespace 必須和「啟動 PX4 時的 PX4_UXRCE_DDS_NS 環境變數」完全一致，
  否則你會訂閱到一個根本不存在的 topic，然後永遠卡在 WAIT_FOR_FCU。

【兩種用法】

 (A) PX4 用預設方式啟動（topic 是 /fmu/...，沒有前綴）：
       終端2:  make px4_sitl gz_x500
       終端4:  ros2 launch drone_control single_drone.launch.py
     → 預設 use_namespace:=false，程式會訂閱 /fmu/...

 (B) PX4 帶 MAV1 名稱啟動（topic 是 /MAV1/fmu/...）：
       終端2:  PX4_UXRCE_DDS_NS=MAV1 make px4_sitl gz_x500
       終端4:  ros2 launch drone_control single_drone.launch.py use_namespace:=true
     → 程式會訂閱 /MAV1/fmu/...

 三機編隊一律走 (B) 的模式，見 three_drones.launch.py。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name', default_value='MAV1',
        description='這台飛機的名字，只影響 log 顯示')

    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description="PX4 是否帶 PX4_UXRCE_DDS_NS 啟動。"
                    "false → 訂閱 /fmu/...；true → 訂閱 /<vehicle_name>/fmu/...")

    target_system_arg = DeclareLaunchArgument(
        'target_system', default_value='1',
        description='MAVLink system id。PX4 的 MAV_SYS_ID = instance + 1，'
                    '所以 instance 0 (MAV1) = 1')

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude', default_value='2.0',
        description='起飛高度（公尺，正值；程式內部會轉成 NED 的負 z）')

    hover_duration_arg = DeclareLaunchArgument(
        'hover_duration', default_value='10.0',
        description='到達高度後懸停幾秒')

    # use_namespace 為 true 時，px4_namespace = "/" + vehicle_name；否則為空字串
    px4_namespace = PythonExpression([
        "'/' + '", LaunchConfiguration('vehicle_name'), "' if '",
        LaunchConfiguration('use_namespace'), "' == 'true' else ''"
    ])

    offboard_node = Node(
        package='drone_control',
        executable='offboard_takeoff_node',
        # ROS 節點也放進以機名命名的 namespace，這樣 ros2 node list 一眼看得出是哪台。
        # 注意：程式裡的 topic 名稱都是絕對路徑（開頭有 /），
        #       所以 ROS namespace 不會影響 topic 訂閱，兩者互不干擾。
        namespace=LaunchConfiguration('vehicle_name'),
        name='offboard_takeoff',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'vehicle_name':       LaunchConfiguration('vehicle_name'),
            'px4_namespace':      px4_namespace,
            'target_system':      LaunchConfiguration('target_system'),
            'takeoff_altitude':   LaunchConfiguration('takeoff_altitude'),
            'hover_duration':     LaunchConfiguration('hover_duration'),
            'position_tolerance': 0.3,
        }],
    )

    return LaunchDescription([
        vehicle_name_arg,
        use_namespace_arg,
        target_system_arg,
        takeoff_altitude_arg,
        hover_duration_arg,
        offboard_node,
    ])
