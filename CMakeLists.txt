cmake_minimum_required(VERSION 3.8)
project(drone_control)

# ROS 2 Humble 用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 依賴 -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# --- 執行檔 -----------------------------------------------------------------
add_executable(offboard_takeoff_node
  src/offboard_takeoff_node.cpp
)

# include/ 目錄放 header，這樣之後寫編隊節點可以直接 #include "drone_control/..."
target_include_directories(offboard_takeoff_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_takeoff_node
  rclcpp
  px4_msgs
)

# --- PX4 <-> ROS 座標翻譯（S1 / S2）-----------------------------------------
# 兩支是同一條轉換鏈的正反向：
#   px4_tf_node         PX4 -> ROS   「我在哪」  發布 TF
#   cmd_vel_to_px4_node ROS  -> PX4  「往那走」  收 /cmd_vel
# 有了這兩支，PX4 對 ROS 來說就像一台普通的 ROS 機器人，Nav2 不需要知道底下是 PX4。
add_executable(px4_tf_node src/px4_tf_node.cpp)
ament_target_dependencies(px4_tf_node
  rclcpp px4_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(cmd_vel_to_px4_node src/cmd_vel_to_px4_node.cpp)
ament_target_dependencies(cmd_vel_to_px4_node
  rclcpp px4_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# S5 的小工具：把 RViz 的「2D Goal Pose」翻譯成呼叫 planner_server。
# bt_navigator 上線（S6）之後就不需要它了。
add_executable(goal_to_planner_node src/goal_to_planner_node.cpp)
ament_target_dependencies(goal_to_planner_node
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs)

# --- 安裝 -------------------------------------------------------------------
install(TARGETS offboard_takeoff_node px4_tf_node cmd_vel_to_px4_node
  goal_to_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# config 要裝到 share/ 底下，參數檔才找得到
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch 檔要裝到 share/ 底下，ros2 launch 才找得到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 啟動多機 SITL 的 shell script
install(PROGRAMS
  test/t_bridge_check.py
  test/t_costmap_check.py
  test/t_planner_check.py
  test/t_nav2_flight_check.py
  test/t_mission_check.py
  scripts/arm_and_takeoff.py
  scripts/check_mocap_axes.py
  scripts/mission_node.py
  scripts/start_3_px4.sh
  scripts/check_env.sh
  scripts/record_flight.sh
  DESTINATION lib/${PROJECT_NAME}
)

# header 也裝起來，之後其他 package 想複用這個類別時可以找到
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 這是個人專案，先關掉 copyright/lint 檢查免得編譯時一堆警告
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
