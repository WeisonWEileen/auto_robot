cmake_minimum_required(VERSION 3.8)
project(rc_state_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} SHARED
   src/rc_state_collector.cpp
)
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   DIRECTORY src/rc_state_collector.cpp
# )

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN rc_state_collector::NavToPoseClient
#   EXECUTABLE navtoposeCli_node
# )

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_state_collector::StateCollectorNode
  EXECUTABLE statecollectorNode)

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

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
