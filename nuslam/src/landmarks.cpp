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

        // Check if this is the first point in the scan
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


    // Detect circles in grouped points
    detect_circles();

    // Publish obstacle locations based on detected circles

  }

  /// @brief Detects circles in groups of lidar points
  void detect_circles() 
  {
    // Iterate through groups vector to analyze each group individually 
    for (int i = 0; i < static_cast<int>(groups.size()); i++) {
        auto group = groups.at(i);
        double x_sum, y_sum, z_bar, 
        turtlelib::Point2D center;
        arma::mat data_matrix(group.size(), 4);
        arma::mat constraint_matrix(4, 4);
        arma::mat constraint_matrix_inv(4, 4);

        // Add points in group, the divide to find mean
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
        constraint_matrix_inv = constraint_matrix.i();

        // Compute singular value decomposition of data matrix
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U, s, V, data_matrix);
        
        // Find smallest singular value of SVD
        double smallest_singular = s(3);
        
        arma::vec A;
        if (smallest_singular < 1e-12){
            // If smallest singular value is lower than 10^-12, A = 4th column of V matrix
            A = V.col(3);
        } else {
            // Create sigma matrix from s vector
            arma::mat sigma(4, 4, arma::fill::zeros);
            for (int i = 0; i < 4; i++){
                sigma.at(i,i) = s.at(i);
            }

            // Calculate Y and Q matrices
            arma::mat Y = V * sigma * V.t();
            arma::mat Q = Y * constraint_mat_inv * Y;
            
            // Find the eigenvectors and values of Q
            arma::cx_vec eigenvalues;
            arma::cx_mat eigenvectors;
            arma::eig_gen(eigenvalues, eigenvectors, Q);

            // Find the smallest positive eigenvalue of Q
            double smallest_pos_eigenvalue = 0.0;
            double smallest_pos_eigenvalue_index = 0.0;
            for (int i = 0; i < 4; i++){
                // Iterate through eigenvalues to find smallest
                const auto curr_eigenvalue = eigenvalues.at(i).real();
                if ((curr_eigenvalue >= 0) && ((curr_eigenvalue < smallest_pos_eigenvalue) ||  (smallest_pos_eigenvalue == -1.0))){
                    smallest_pos_eigenvalue = curr_eigenvalue;
                    smallest_pos_eigenvalue_index = i;
                }
            }
            
            // Astar is eigenvector corresponding to the smallest positive eigenvalue of Q
            arma::cx_vec Astar = eigenvectors.col(smallest_pos_eigenvalue_index);

            // Find A from Y * A = Astar
            arma::cx_mat A_cx = Y.i() * Astar;
            
            // Use real values of A from A_cx
            A = arma::vec(4);
            for (int i = 0; i < 4; i++){
                A.at(i) = A_cx.at(i).real();
            }
        }
        
        // Extract A values from A vector
        double A1 = A.at(0);
        double A2 = A.at(1);
        double A3 = A.at(2);
        double A4 = A.at(3);

        // Find coordinates and radius of circle using A values
        double a = -0.5 * A2 / A1;
        double b = -0.5 * A3 / A1;
        double rad = sqrt(((A2 * A2) + (A3 * A3) - (4 * A1 * A4))/(4 * A1 * A1));    

        // Check if group is a circle 
    }

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
