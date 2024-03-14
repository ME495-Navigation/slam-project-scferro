/// \file landmarks.cpp
/// \brief Detects circles in lidar scan data
///
/// PARAMETERS:
///     use_real_lidar (bool): subscribe to real or simulated lidar data
/// PUBLISHES:
///     detected_obs (visualization_msgs::msg::MarkerArray): circle detection results
/// SUBSCRIBES:
///    scan (sensor_msgs::msg::LaserScan): lidar scan data


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/qos.hpp"


class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Parameters and default values
    declare_parameter("rate", 200);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();

    // Other variables


    // Publishers
    obstacles_pub = create_publisher<visualization_msgs::msg::MarkerArray>("detected_obs", 10);

    // Subscribers
    laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10, std::bind(&Landmarks::laser_callback, this, std::placeholders::_1));
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, pose_rate, fake_sensor_rate;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circles_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

  /// \brief The wheel_cmd callback function, updates wheel speeds and robot ground truth position
  void laser_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Landmarks>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
