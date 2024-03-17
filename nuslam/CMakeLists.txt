cmake_minimum_required(VERSION 3.8)
project(nuslam)

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
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(catch_ros2 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(nuslam src/nuslam.cpp)
ament_target_dependencies(nuslam rclcpp std_msgs geometry_msgs sensor_msgs
  std_srvs tf2 tf2_ros visualization_msgs nuturtlebot_msgs nav_msgs Armadillo)
include_directories(${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(nuslam turtlelib::turtlelib ${ARMADILLO_LIBRARIES} "${cpp_typesupport_target}")

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks rclcpp std_msgs geometry_msgs sensor_msgs
  visualization_msgs nuturtlebot_msgs nav_msgs Armadillo)
include_directories(${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(landmarks turtlelib::turtlelib ${ARMADILLO_LIBRARIES} "${cpp_typesupport_target}")

add_executable(circle_test test/circle_test.cpp)
ament_target_dependencies(circle_test rclcpp std_msgs geometry_msgs sensor_msgs
  visualization_msgs nuturtlebot_msgs nav_msgs Armadillo)
target_link_libraries(circle_test catch_ros2::catch_ros2_with_node_main)


install(TARGETS
  nuslam
  landmarks
  circle_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  test
  DESTINATION share/${PROJECT_NAME}/
)

catch_ros2_add_integration_test(circle_test LAUNCH_FILE circle_test.launch.py)

find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()


ament_package()
