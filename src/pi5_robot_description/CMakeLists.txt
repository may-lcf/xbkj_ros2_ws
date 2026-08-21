cmake_minimum_required(VERSION 3.8)
project(pi5_robot_description)

find_package(ament_cmake REQUIRED)

# 安装 URDF、launch、config、meshes 等文件
install(DIRECTORY
  urdf
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本
install(PROGRAMS
  pi5_robot_description/odom_only_node.py
  pi5_robot_description/cmd_vel_bridge_node.py
  pi5_robot_description/goal_pose_bridge.py
  pi5_robot_description/waypoint_navigator.py
  pi5_robot_description/nav_executor_node.py
  pi5_robot_description/yolo_zone_detect_node.py
  pi5_robot_description/arm_fixed_pose_node.py
  pi5_robot_description/depth_convert_node.py
  pi5_robot_description/arm_kinematics.py
  pi5_robot_description/arm_serial.py
  pi5_robot_description/depth_utils.py
  pi5_robot_description/color_detector.py
  pi5_robot_description/nav_pick_place_node.py
  pi5_robot_description/waypoint_recorder_node.py
  scripts/save_map.sh
  scripts/verify_urdf.sh
  scripts/waypoint_control.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
