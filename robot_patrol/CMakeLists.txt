cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(robot_state_publisher REQUIRED)
#find_package(robot_model REQUIRED)
#find_package(visualization_msgs REQUIRED)

# ========= EXECUTABLES ==========
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node 
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service 
	rclcpp 
	sensor_msgs
	robot_interfaces
)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service 
    rclcpp 
    sensor_msgs
    robot_interfaces
)

# ========= INSTALL ==========
install(TARGETS 
  patrol_node
  direction_service
  test_service
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

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



ament_package()
