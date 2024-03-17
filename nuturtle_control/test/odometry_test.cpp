#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
// #include "nuturtle_control/srv/pose.hpp"

using namespace std::chrono_literals;

auto tf_found{false};
auto srv_found{false};

TEST_CASE("odom->base_footprint transform", "[odometry]") {
  auto node = rclcpp::Node::make_shared("odometry_test_node");

  // Set test duration
  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Create TF buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer =
    std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::Time start_time = rclcpp::Clock().now();

  // Until test duration expires
  while (rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    // Try to get the odom-> base footprint TF
    try {
      // Return true if tf is found
      auto transform = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      tf_found = true;
    } catch (const tf2::TransformException & ex) {
      return;
    }

    // Spin node again until time expires
    rclcpp::spin_some(node);
  }

  // Test if TF is found
  CHECK(tf_found);
}

// TEST_CASE("Initial pose server", "[odometry]") {
//   auto node = rclcpp::Node::make_shared("odometry_test_node");
//   srv_found = false;
//   tf_found = false;

//   // Set test duration
//   node->declare_parameter<double>("test_duration");
//   const auto TEST_DURATION =
//     node->get_parameter("test_duration").get_parameter_value().get<double>();

//   // Creat initial pose client
//   auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

//   rclcpp::Time start_time = rclcpp::Clock().now();

//   // Until test ends, try to call service
//   while (rclcpp::ok() &&
//     ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
//   {
//     if (client->wait_for_service(0s)) {
//       srv_found = true;
//       break;

//       // Make a service request and send it
//       auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
//       request->x = 3.0;
//       request->y = 1.5;
//       request->theta = 3.1415926535;
//       auto result = client->async_send_request(request);

//       // If you were able to get the transform, check that the robot moved to the correct position
//       if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
//         // Mark that service was called
//         srv_found = true;

//         // Check if result was correct
//         if (result.get()->success) {
//           try {
//             // Get current robot TF
//             std::unique_ptr<tf2_ros::Buffer> tf_buffer =
//               std::make_unique<tf2_ros::Buffer>(node->get_clock());
//             std::shared_ptr<tf2_ros::TransformListener> tf_listener =
//               std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
//             auto t = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

//             // Note that TF was found
//             tf_found = true;

//             // Check if TF matches what was sent
//             REQUIRE_THAT(t.transform.translation.x, Catch::Matchers::WithinAbs(3.0, 1e-5));
//             REQUIRE_THAT(t.transform.translation.y, Catch::Matchers::WithinAbs(1.5, 1e-5));
//             REQUIRE_THAT(t.transform.rotation.z, Catch::Matchers::WithinAbs(3.1415926535, 1e-5));
//           } catch (const tf2::TransformException & ex) {
//             return;
//           }
//         }
//       }
//       break;
//     }
//     rclcpp::spin_some(node);
//   }

//   // Test if service could be called
//   CHECK(srv_found);
//   // CHECK(tf_found);
// }
