# Install script for directory: /home/webotpi/ros2_ws/src/deptrum-ros-driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/webotpi/ros2_ws/install/deptrum-ros-driver-aurora930")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930" TYPE EXECUTABLE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/aurora930_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node"
         OLD_RPATH "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/lib:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/glog-0.4.0/lib-static:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/gflags-2.2.2/lib-static:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/CMakeFiles/aurora930_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930" TYPE EXECUTABLE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/subscribe_node/sub_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node"
         OLD_RPATH "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/lib:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/glog-0.4.0/lib-static:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/gflags-2.2.2/lib-static:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/subscribe_node/CMakeFiles/sub_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930" TYPE EXECUTABLE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/aurora930_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node"
         OLD_RPATH "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/lib:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/glog-0.4.0/lib-static:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/gflags-2.2.2/lib-static:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/aurora930_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/CMakeFiles/aurora930_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930" TYPE EXECUTABLE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/subscribe_node/sub_node_ci")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci"
         OLD_RPATH "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/lib:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/glog-0.4.0/lib-static:/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/gflags-2.2.2/lib-static:/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/deptrum-ros-driver-aurora930/sub_node_ci")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/subscribe_node/CMakeFiles/sub_node_ci.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/scripts/99-deptrum-libusb.rules")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "/home/webotpi/ros2_ws/src/deptrum-ros-driver/ext/deptrum-stream-aurora900-linux-aarch64-v1.1.19-18.04/lib/libdeptrum_stream_aurora900.so.1.1.19")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE DIRECTORY FILES "/home/webotpi/ros2_ws/src/deptrum-ros-driver/launch_aurora930/launch" FILES_MATCHING REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/doc" TYPE FILE FILES
    "/home/webotpi/ros2_ws/src/deptrum-ros-driver/docs/usage_aurora930.md"
    "/home/webotpi/ros2_ws/src/deptrum-ros-driver/RELEASE_AURORA930.md"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/rviz" TYPE FILE FILES "/home/webotpi/ros2_ws/src/deptrum-ros-driver/rviz/aurora930-ros2.rviz")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/deptrum-ros-driver-aurora930")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/deptrum-ros-driver-aurora930")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/environment" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_index/share/ament_index/resource_index/packages/deptrum-ros-driver-aurora930")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/cmake" TYPE FILE FILES "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930/cmake" TYPE FILE FILES
    "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_core/deptrum-ros-driver-aurora930Config.cmake"
    "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/ament_cmake_core/deptrum-ros-driver-aurora930Config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deptrum-ros-driver-aurora930" TYPE FILE FILES "/home/webotpi/ros2_ws/src/deptrum-ros-driver/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/subscribe_node/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/webotpi/ros2_ws/build/deptrum-ros-driver-aurora930/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
