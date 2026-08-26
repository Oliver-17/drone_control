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

# --- 安裝 -------------------------------------------------------------------
install(TARGETS offboard_takeoff_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 檔要裝到 share/ 底下，ros2 launch 才找得到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 啟動多機 SITL 的 shell script
install(PROGRAMS
  scripts/start_3_px4.sh
  scripts/check_env.sh
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
