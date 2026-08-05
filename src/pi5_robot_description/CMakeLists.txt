cmake_minimum_required(VERSION 3.8)
project(pi5_robot_description)

find_package(ament_cmake REQUIRED)

# 安装 URDF、launch、config 等文件
install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本
install(PROGRAMS
  pi5_robot_description/odom_only_node.py
  pi5_robot_description/cmd_vel_bridge_node.py
  pi5_robot_description/goal_pose_bridge.py
  scripts/save_map.sh
  scripts/verify_urdf.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
