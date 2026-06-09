cmake_minimum_required(VERSION 3.8)
project(my_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbridge_suite REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/Add.srv"
)

file(GLOB MY_SRV_SCRIPTS scripts/*.py)
install(PROGRAMS ${MY_SRV_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY scripts/
#   # scripts/color_sorting_node.py  # 脚本路径
#   scripts/color_stack_node.py  # 脚本路径
#   # scripts/test.py
#   DESTINATION lib/${PROJECT_NAME}  
# )

# 安装 Launch 文件到 share/<package_name>/launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# 安装前端 HTML 页面到 share/<package_name>/web（供静态文件服务器托管）
install(DIRECTORY web/
        DESTINATION share/${PROJECT_NAME}/web)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rosbridge_suite  # 导出 rosbridge_suite，确保其他包能找到它
  cv_bridge
  rosidl_default_generators
)

ament_package()
