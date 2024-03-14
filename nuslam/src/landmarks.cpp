/// \file landmarks.cpp
/// \brief Detects circles in lidar scan data
///
/// PARAMETERS:
///     use_real_lidar (bool): subscribe to real or simulated lidar data
///     grouping_threshold (bool): threshold value for creating groups of points from lidar scan
/// PUBLISHES:
///     detected_obs (visualization_msgs::msg::MarkerArray): circle detection results
/// SUBSCRIBES:
///    scan (sensor_msgs::msg::LaserScan): lidar scan data


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/qos.hpp"
#include "turtlelib/geometry2d.hpp"


class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Parameters and default values
    declare_parameter("rate", 200);
    declare_parameter("group_threshold", 0.1);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    group_threshold = get_parameter("group_threshold").as_double();

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
  int loop_rate;
  double group_threshold;
  builtin_interfaces::msg::Time last_msg_time;
  std::vector<std::vector<turtlelib::Point2D>> groups;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

  /// \brief The wheel_cmd callback function, updates wheel speeds and robot ground truth position
  void laser_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<turtlelib::Point2D> current_group;
    turtlelib::Point2D prev_point;
    bool first_point_flag = true;
    turtlelib::Point2D current_point, first_point;

    // Clear groups
    groups.clear();
    
    for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++) {
        // Extract heading and angle and convert to cartesian
        double angle = msg.angle_min + ((i / msg.ranges.size()) * 2 * 3.1415926);
        double rad = msg.ranges.at(i);
        current_point = polar_to_cart(angle, rad);

        if (!first_point_flag) {

            // Calculate distance to previous point
            double dist_to_prev = sqrt(pow(current_point.x - prev_point.x, 2) + pow(current_point.y - prev_point.y, 2));

            // If distance is less than threshold, add point to current group
            // Else, create a new group and add old group to groups vector
            if (dist_to_prev > group_threshold) {
                
                // Add group to group vector then reset current_group
                groups.push_back(current_group);
                current_group.clear();
            }

            // Add current point current_group
            current_group.push_back(current_point);
        } else {
            // set first point flag to false now that a previous point will be stored
            first_point_flag = false;
            
            // Store first point
            first_point = current_point;

            // Add current point current_group
            current_group.push_back(current_point);
        }

        prev_point = current_point;
    }

    // Find distance from last point to first point
    double dist_first_last = sqrt(pow(current_point.x - first_point.x, 2) + pow(current_point.y - first_point.y, 2));

    // Check if last group belongs should be added to first group
    // Else combine last and first groups
    if (dist_first_last > group_threshold) {
        // Add group to group vector then reset current_group
        groups.push_back(current_group);
    } else {
        // Combine first and last groups
        groups.at(0).insert(groups.at(0).begin(), current_group.begin(), current_group.end());
    }

    // RCLCPP_INFO(this->get_logger(), "groups: %ld", groups.size());

    
  }
  
  /// @brief  Converts polar coordinates to a point
  /// @param angle The angle of the polar coordinates
  /// @param rad The radius of the polar coordinates
  /// @return point in Point2D
  turtlelib::Point2D polar_to_cart(double angle, double rad)
  {
    turtlelib::Point2D point;
    point.x = rad * cos(angle);
    point.y = rad * sin(angle);

    return point;
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
