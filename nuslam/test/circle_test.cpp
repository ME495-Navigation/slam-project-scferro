// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "catch_ros2/catch_ros2.hpp"

// using namespace std::chrono_literals;

// // Testing cmd_vel commands for driving straight/pure translation
// TEST_CASE("Test Circle Center Detection", "[turtle_control]")
// {
//   sub_found = false;
//   auto node = rclcpp::Node::make_shared("turtle_control_test");
//   rclcpp::Time start_time = rclcpp::Clock().now();

//   // Declare parameter
//   node->declare_parameter("test_duration", 5.0);
//   const auto test_duration =
//     node->get_parameter("test_duration").get_parameter_value().get<double>();

//   // Initialize message
//   geometry_msgs::msg::Twist twist_msg;
//   twist_msg.linear.x = 0.1;
//   twist_msg.angular.z = 0.0;

//   // Create publisher and subscriber
//   auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
//   auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
//     "wheel_cmd", 10, &wheel_cmd_callback);

//   // Wait until message received or time exceeds test_duration parameter
//   while (rclcpp::ok() && !(sub_found) &&
//     ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(test_duration)))
//   {
//     cmd_vel_pub->publish(twist_msg);
//     rclcpp::spin_some(node);
//   }

//   // Check if command received matches expectation
//   REQUIRE_THAT(left_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
//   REQUIRE_THAT(right_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
// }