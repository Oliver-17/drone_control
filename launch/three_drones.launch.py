#!/usr/bin/env python3
"""
three_drones.launch.py — 三機同時起飛（MAV1 長機 + MAV2/MAV3 僚機）

【前置條件】
  必須先用 scripts/start_3_px4.sh 啟動三個 PX4 SITL instance，
  它們的 PX4_UXRCE_DDS_NS 分別是 MAV1 / MAV2 / MAV3。

【三台的對應關係】（依據 PX4 v1.14 的 ROMFS/px4fmu_common/init.d-posix/rcS）

  角色    instance   PX4_UXRCE_DDS_NS   topic 前綴     MAV_SYS_ID    起飛位置(N,E)
  ------  ---------  -----------------  -------------  ------------  ------------
  MAV1     -i 0       MAV1               /MAV1/fmu/...  1             ( 0,  0)   長機
  MAV2     -i 1       MAV2               /MAV2/fmu/...  2             ( 0,  3)   右僚機
  MAV3     -i 2       MAV3               /MAV3/fmu/...  3             ( 0, -3)   左僚機

  MAV_SYS_ID 來自 rcS:131 的 `param set MAV_SYS_ID $((px4_instance+1))`，
  所以 target_system 一定是 instance + 1，不能填錯，否則指令會送給別台飛機。

【目前的行為】
  三台各自獨立跑完「起飛 → 懸停 → 降落」，彼此不溝通。
  高度刻意錯開（2.0 / 2.5 / 3.0 m），避免萬一水平漂移時互撞。
  真正的編隊邏輯（互相追隨、保持隊形）是下一步，還沒實作。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (機名, target_system, 起飛高度)
FLEET = [
    ('MAV1', 1, 2.0),   # 長機
    ('MAV2', 2, 2.5),   # 僚機
    ('MAV3', 3, 3.0),   # 僚機
]


def make_node(vehicle_name: str, target_system: int, altitude: float) -> Node:
    return Node(
        package='drone_control',
        executable='offboard_takeoff_node',
        namespace=vehicle_name,
        name='offboard_takeoff',
        output='both',      # 'both' = 螢幕與檔案都輸出（檔案存在 ~/.ros/log/）
        emulate_tty=True,
        parameters=[{
            'vehicle_name':       vehicle_name,
            # 前綴必須與 PX4 啟動時的 PX4_UXRCE_DDS_NS 一致
            'px4_namespace':      f'/{vehicle_name}',
            'target_system':      target_system,
            'takeoff_altitude':   altitude,
            'hover_duration':     10.0,
            # 可從命令列覆寫，低空測試時要調小
            'position_tolerance': LaunchConfiguration('position_tolerance'),
        }],
    )


def generate_launch_description():
    position_tolerance_arg = DeclareLaunchArgument(
        'position_tolerance', default_value='0.3',
        description='高度到達的容忍值（公尺）。低空測試務必調小：'
                    '容忍值若接近起飛高度，飛機還沒爬上去就會被判定「已到達」，'
                    '建議不超過 takeoff_altitude 的三分之一')

    return LaunchDescription([position_tolerance_arg] + [
        make_node(name, sysid, alt) for name, sysid, alt in FLEET
    ])
