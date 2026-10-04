cmake_minimum_required(VERSION 3.8)
project(rm_nav_bringup)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装仿真用点云转激光脚本到 share，便于 launch  reliably 找到
install(PROGRAMS scripts/pointcloud_to_laserscan_sim.py DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/odom_to_tf.py DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS
  scripts/odin_grid_map.py
  scripts/finish_odin_mapping.py
  scripts/wait_odin_localization.py
  scripts/odin_foxglove_control.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  map
  PCD
  rviz
  urdf
)
