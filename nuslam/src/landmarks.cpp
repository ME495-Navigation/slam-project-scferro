/// \file landmarks.cpp
/// \brief Detects circles in lidar scan data
///
/// PARAMETERS:
///     group_threshold (bool): threshold value for creating groups of points from lidar scan
///     std_dev_thresh (double): threshold value for angle std dev when detecting circles
///     mean_lo_thresh (double): lower threshold value for angle mean when detecting circles
///     mean_hi_thresh (double): upper threshold value for angle mean when detecting circles
/// PUBLISHES:
///     detected_obs (visualization_msgs::msg::MarkerArray): circle detection results
/// SUBSCRIBES:
///     scan (sensor_msgs::msg::LaserScan): lidar scan data


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "turtlelib/geometry2d.hpp"
#include "builtin_interfaces/msg/time.hpp"

/// @brief The landmarks node
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Parameters and default values
    declare_parameter("group_threshold", 0.25);
    declare_parameter("std_dev_thresh", 0.4);
    declare_parameter("mean_lo_thresh", 1.4);
    declare_parameter("mean_hi_thresh", 2.6);

    // Define parameter variables
    group_threshold = get_parameter("group_threshold").as_double();
    std_dev_thresh = get_parameter("std_dev_thresh").as_double();
    mean_lo_thresh = get_parameter("mean_lo_thresh").as_double();
    mean_hi_thresh = get_parameter("mean_hi_thresh").as_double();

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
  double group_threshold, std_dev_thresh, mean_lo_thresh, mean_hi_thresh;
  std::vector<std::vector<turtlelib::Point2D>> groups;
  std::vector<turtlelib::Circle2D> obstacle_list;
  builtin_interfaces::msg::Time msg_stamp;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

  /// @brief The wheel_cmd callback function, updates wheel speeds and robot ground truth position
  /// @param msg (sensor_msgs::msg::LaserScan) the laser scan message 
  void laser_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<turtlelib::Point2D> current_group;
    turtlelib::Point2D prev_point;
    bool first_point_flag = true;
    turtlelib::Point2D current_point, first_point;
    msg_stamp = msg.header.stamp;

    // Clear groups and obstacle lists
    groups.clear();
    obstacle_list.clear();

    for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++) {
      // Extract heading and angle and convert to cartesian
      double angle = msg.angle_min + (i * 2 * 3.1415926 / msg.ranges.size());
      double rad = msg.ranges.at(i);
      current_point = polar_to_cart(angle, rad);

      // Check if this is the first point in the scan
      if (!first_point_flag) {
        // Calculate distance to previous point
        double dist_to_prev =
          sqrt(pow(current_point.x - prev_point.x, 2) + pow(current_point.y - prev_point.y, 2));
        // If distance is greater than threshold, create a new group and add old group to groups vector
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
    double dist_first_last =
      sqrt(pow(current_point.x - first_point.x, 2) + pow(current_point.y - first_point.y, 2));

    // Check if last group belongs should be added to first group
    // Else combine last and first groups
    if (dist_first_last > group_threshold) {
      // Add group to group vector then reset current_group
      groups.push_back(current_group);
    } else {
      // Combine first and last groups
      groups.at(0).insert(groups.at(0).begin(), current_group.begin(), current_group.end());
    }

    // Detect circles in grouped points
    detect_circles();

    // RCLCPP_INFO(this->get_logger(), "Obstacle Count: %ld", obstacle_list.size());

    // Publish obstacle locations based on detected circles
    publish_obstacles();

  }

  /// @brief Publish marker array with obstacle positions and sizes
  void publish_obstacles()
  {
    // Create marker array fill with markers
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i = 0; i < static_cast<int>(obstacle_list.size()); i++) {
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = msg_stamp;
      marker.header.frame_id = "green/base_footprint";
      marker.id = i;
      marker.type = 3;  // cylinder
      marker.action = 0;

      // Set color to green
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;

      // Set Radius
      marker.scale.x = 2 * obstacle_list.at(i).rad;
      marker.scale.y = 2 * obstacle_list.at(i).rad;
      marker.scale.z = 0.25;

      // Set the obstacle location relative to the robot frame
      marker.pose.position.x = obstacle_list.at(i).x;
      marker.pose.position.y = obstacle_list.at(i).y;
      marker.pose.position.z = 0.125;

      // Add to marker array
      marker_array.markers.push_back(marker);
    }

    // Publish obstacle markers
    obstacles_pub->publish(marker_array);
  }

  /// @brief Detects circles in groups of lidar points
  void detect_circles()
  {
    // Iterate through groups vector to analyze each group individually
    for (int i = 0; i < static_cast<int>(groups.size()); i++) {
      auto group = groups.at(i);
      turtlelib::Point2D center;
      arma::mat data_matrix = arma::zeros(group.size(), 4);
      arma::mat constraint_matrix = arma::zeros(4, 4);

      // Check if group is a circle
      bool is_circle = circle_check(group);

      // If group is circle, find center and radius and add to obstacle vector
      if (is_circle) {

        // Add points in group, then divide to find mean. mean is center of circle
        double x_sum = 0.;
        double y_sum = 0.;
        double z_bar = 0.;
        for (int j = 0; j < static_cast<int>(group.size()); j++) {
          x_sum += group.at(j).x;
          y_sum += group.at(j).y;
        }
        center.x = x_sum / group.size();
        center.y = y_sum / group.size();

        // Iterate through all points in group
        for (int j = 0; j < static_cast<int>(group.size()); j++) {
          // Extract next point
          turtlelib::Point2D point = group.at(j);

          // Adjust x and y with center
          double x_i = point.x - center.x;
          double y_i = point.y - center.y;
          double z_i = pow(x_i, 2) + pow(y_i, 2);

          // Add data to data matrix
          data_matrix.at(j, 0) = z_i;
          data_matrix.at(j, 1) = x_i;
          data_matrix.at(j, 2) = y_i;
          data_matrix.at(j, 3) = 1;

          // Add z_i to z_bar
          z_bar += z_i;
        }

        // Calculate z_bar, average of z_i
        z_bar = z_bar / group.size();

        // Fill out constraint matrix and inverse
        constraint_matrix.at(0, 0) = 8. * z_bar;
        constraint_matrix.at(1, 1) = 1.;
        constraint_matrix.at(2, 2) = 1.;
        constraint_matrix.at(3, 0) = 2.;
        constraint_matrix.at(0, 3) = 2.;

        // Compute singular value decomposition of data matrix
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U, s, V, data_matrix);

        // Find smallest singular value of SVD
        double smallest_singular = s(3);

        arma::colvec A;
        if (smallest_singular < 1e-12) {
          // If smallest singular value is lower than 10^-12, A = 4th column of V matrix
          A = V.col(3);
        } else {
          // Calculate Y and Q matrices
          arma::mat Y = V * arma::diagmat(s) * V.t();
          arma::mat Q = Y * constraint_matrix.i() * Y;

          // Find the eigenvectors and values of Q
          arma::cx_vec eigenvalues;
          arma::cx_mat eigenvectors;
          arma::eig_gen(eigenvalues, eigenvectors, Q);
          // Get real values
          arma::mat real_eigenvalues = arma::real(eigenvalues);
          arma::mat real_eigenvectors = arma::real(eigenvectors);

          // Find the smallest positive eigenvalue of Q
          double smallest_pos_eigenvalue = -1.0;
          double smallest_pos_eigenvalue_index = 0.0;
          for (int i = 0; i < static_cast<int>(real_eigenvalues.size()); i++) {
            // Iterate through eigenvalues to find smallest
            auto curr_eigenvalue = real_eigenvalues.at(i);
            if ((curr_eigenvalue >= 0) &&
              ((curr_eigenvalue < smallest_pos_eigenvalue) || (smallest_pos_eigenvalue == -1.0)))
            {
              smallest_pos_eigenvalue = curr_eigenvalue;
              smallest_pos_eigenvalue_index = i;
            }
          }

          // A_star is eigenvector corresponding to the smallest positive eigenvalue of Q
          arma::vec A_star = real_eigenvectors.col(smallest_pos_eigenvalue_index);

          // Find A from Y * A = A_star
          A = Y.i() * A_star;
        }

        // Extract A values from A vector
        double A1 = A.at(0);
        double A2 = A.at(1);
        double A3 = A.at(2);
        double A4 = A.at(3);

        // Find coordinates and radius of circle using A values
        double a = -0.5 * A2 / A1;
        double b = -0.5 * A3 / A1;
        double rad = sqrt((pow(A2, 2) + pow(A3, 2) - (4. * A1 * A4)) / (4. * pow(A1, 2)));

        // Create obstacle Circle2D object and add obstacle to obstacle list
        turtlelib::Circle2D obstacle;
        obstacle.x = a + center.x;
        obstacle.y = b + center.y;
        obstacle.rad = rad;
        obstacle_list.push_back(obstacle);
      }
    }
  }

  /// @brief Checks if group of points is a circle
  /// @param group (std::vector<turtlelib::Point2D>)group of Point2D points from a laser scan
  /// @return true if points are circle, false if not
  bool circle_check(std::vector<turtlelib::Point2D> group)
  {
    turtlelib::Point2D P1 = group.at(0);
    turtlelib::Point2D P2 = group.at(group.size() - 1);
    std::vector<double> angles;
    double angles_mean = 0.0;
    double angles_std_dev = 0.0;
    double c = sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2));

    // If group is smaller than 4 points, not enough data to check if it's a circle
    // If group is larger than 30 points, not a real obstacle, likely the curve of the lidar scan in space
    if (group.size() < 4) {
      return false;
    } else if (group.size() > 30) {
      return false;
    }

    // Find angles between P1, P, and P2 for all P in group
    for (int i = 1; i < static_cast<int>(group.size()) - 2; i++) {
      // Extract current point
      auto P = group.at(i);
      // Solve for angle with law of cosines
      double a = sqrt(pow(P1.x - P.x, 2) + pow(P1.y - P.y, 2));
      double b = sqrt(pow(P.x - P2.x, 2) + pow(P.y - P2.y, 2));
      // Find angle between line a and b
      double numer = pow(a, 2) + pow(b, 2) - pow(c, 2);
      double denom = (2 * a * b);
      double angle = acos(numer / denom);
      // Add angle to mean and std_dev
      angles_mean += angle;
      angles.push_back(angle);
    }

    // Find mean and std dev
    angles_mean = angles_mean / angles.size();
    for (int i = 0; i < (static_cast<int>(angles.size()) - 1); i++) {
      angles_std_dev += ((angles.at(i) - angles_mean) * (angles.at(i) - angles_mean));
    }
    angles_std_dev = sqrt(angles_std_dev / (angles.size()));

    // Return true if angle meets all threshold criteria
    return (angles_std_dev < std_dev_thresh) &&
           (angles_mean > mean_lo_thresh) &&
           (angles_mean < mean_hi_thresh);
  }

  /// @brief  Converts polar coordinates to a point
  /// @param angle (double)The angle of the polar coordinates
  /// @param rad (double) The radius of the polar coordinates
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
