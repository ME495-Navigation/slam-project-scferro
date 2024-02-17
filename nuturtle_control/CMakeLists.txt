cmake_minimum_required(VERSION 3.22)
project(nuturtle_control)

# Used ChatGPT for debugging
# Refer to Citation [5] ChatGPT

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(catch_ros2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/Control.srv"
  "srv/Pose.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")


add_executable(turtle_control  src/turtle_control.cpp)
ament_target_dependencies(turtle_control rclcpp std_msgs geometry_msgs
  sensor_msgs std_srvs nav_msgs tf2 tf2_ros visualization_msgs nuturtlebot_msgs)
target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp std_msgs geometry_msgs sensor_msgs
  std_srvs nav_msgs tf2 tf2_ros visualization_msgs nuturtlebot_msgs)
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(circle src/circle.cpp)
ament_target_dependencies(circle rclcpp std_msgs geometry_msgs sensor_msgs
  std_srvs nav_msgs tf2 tf2_ros visualization_msgs nuturtlebot_msgs)
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(turtle_control_test test/turtle_control_test.cpp)
target_link_libraries(turtle_control_test catch_ros2::catch_ros2_with_node_main)
ament_target_dependencies(turtle_control_test rclcpp nuturtlebot_msgs geometry_msgs catch_ros2 sensor_msgs)

install(TARGETS
  turtle_control
  odometry
  circle
  turtle_control_test
  DESTINATION lib/${PROJECT_NAME}
)

catch_ros2_add_integration_test(turtle_control_test LAUNCH_FILE turtle_control_test.launch.xml)

install(DIRECTORY
  launch
  srv
  config
  test
  DESTINATION share/${PROJECT_NAME}/
)

find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_package()
