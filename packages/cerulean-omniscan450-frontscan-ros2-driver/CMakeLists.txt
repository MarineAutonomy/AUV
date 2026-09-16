cmake_minimum_required(VERSION 3.8)
project(frontscan_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrontScanSonar.msg"
  "msg/FrontScanSonarRaw.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(frontscan_ros2_nodes)
ament_python_install_package(brping)

install(PROGRAMS
  scripts/frontscan_node
  scripts/viewer_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
