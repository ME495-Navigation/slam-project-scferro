#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "catch_ros2/catch_ros2.hpp"

using namespace std::chrono_literals;

// Initialize variables
bool sub_found = false;
double left_wheel_vel = 0.0;
double right_wheel_vel = 0.0;
double left_joint_pos = 0.0;
double right_joint_pos = 0.0;
double left_joint_vel = 0.0;
double right_joint_vel = 0.0;

// Extract the wheel commands from wheel_cmd messages
void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands msg)
{
  left_wheel_vel = msg.left_velocity;
  right_wheel_vel = msg.right_velocity;
}

// Extract the joint states from JointState messages
void joint_states_callback(const sensor_msgs::msg::JointState msg)
{
  left_joint_pos = msg.position[0];
  right_joint_pos = msg.position[1];
  left_joint_vel = msg.velocity[0];
  right_joint_vel = msg.velocity[1];
}

// Testing cmd_vel commands for driving straight/pure translation
TEST_CASE("Pure translation", "[turtle_control]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  // Declare parameter
  node->declare_parameter("test_duration", 5.0);
  const auto test_duration =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Initialize message
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.1;
  twist_msg.angular.z = 0.0;

  // Create publisher and subscriber
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &wheel_cmd_callback);

  // Wait until message received or time exceeds test_duration parameter
  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(test_duration)))
  {
    cmd_vel_pub->publish(twist_msg);
    rclcpp::spin_some(node);
  }

  // Check if command received matches expectation
  REQUIRE_THAT(left_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
  REQUIRE_THAT(right_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
}

// Testing cmd_vel commands for spinning in a circle/pure rotation
TEST_CASE("Pure rotation", "[turtle_control]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  // Declare parameter
  node->declare_parameter("test_duration", 5.0);
  const auto test_duration =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Initialize message
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.25;

  // Create publisher and subscriber
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &wheel_cmd_callback);

  // Wait until message received or time exceeds test_duration parameter
  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(test_duration)))
  {
    cmd_vel_pub->publish(twist_msg);
    rclcpp::spin_some(node);
  }

  // Check if command received matches expectation
  REQUIRE_THAT(left_wheel_vel, Catch::Matchers::WithinAbs(25.0, 1e-5));
  REQUIRE_THAT(right_wheel_vel, Catch::Matchers::WithinAbs(-25.0, 1e-5));
}

// Testing the conversion from sensor_data (ticks) to joint_states (radians)
TEST_CASE("Ticks to radians", "[turtle_control]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  // Declare parameter
  node->declare_parameter("test_duration", 5.0);
  const auto test_duration =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Initialize message
  nuturtlebot_msgs::msg::SensorData sensor_data_msg;
  sensor_data_msg.left_encoder = 100;
  sensor_data_msg.right_encoder = 100;

  // Create publisher and subscriber
  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data", 10);
  auto joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, &joint_states_callback);

  // Wait until message received or time exceeds test_duration parameters
  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(test_duration)))
  {
    sensor_data_pub->publish(sensor_data_msg);
    rclcpp::spin_some(node);
  }

  // Check if command received matches expectation
  REQUIRE_THAT(left_joint_pos, Catch::Matchers::WithinAbs(0.15339, 1e-5));
  REQUIRE_THAT(right_joint_pos, Catch::Matchers::WithinAbs(0.15339, 1e-5));
}
