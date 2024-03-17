/// \file nuslam.cpp
/// \brief SLAM for the turtlebot
///
/// PARAMETERS:
///     rate (double): frequency of the timer (Hz)
/// SUBSCRIBES:
///     ~/red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): the commands for the wheel motors
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): encoder data for the simulated robot
///     green/path (nav_msgs::msg::Path): the path of the nusim robot
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a specified x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <functional>
#include <armadillo>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class Nuslam : public rclcpp::Node
{
public:
  Nuslam()
  : Node("nuslam")
  {
    // Parameters and default values
    declare_parameter("rate", 2);
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_diameter", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("max_obs", 10);
    declare_parameter("obstacle_radius", 0.038);
    declare_parameter("obs_dist_thresh", 0.4);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    track_width = get_parameter("track_width").as_double();
    max_obs = get_parameter("max_obs").as_int();
    obstacle_radius = get_parameter("obstacle_radius").as_double();
    obs_dist_thresh = get_parameter("obs_dist_thresh").as_double();

    // Check if parameters have been defined. if not, throw runtime error
    // Refer to Citation [2] ChatGPT
    if (wheel_diameter == -1.0 || track_width == -1.0) {
      throw std::runtime_error("Diff drive parameters not correctly defined.");
    }
    if (body_id == "" || wheel_left == "" || wheel_right == "") {
      throw std::runtime_error("Frame parameters not correctly defined.");
    }

    // Create diff_drive, initialize wheel positions
    diff_drive = turtlelib::DiffDrive(wheel_diameter, track_width);
    left_wheel_angle = 0.;
    right_wheel_angle = 0.;
    tf_odom_body = turtlelib::Transform2D{{0.0, 0.0}, 0.0};
    tf_map_odom = turtlelib::Transform2D{{0.0, 0.0}, 0.0};
    tf_map_body = turtlelib::Transform2D{{0.0, 0.0}, 0.0};
    path_counter = 0;
    num_obs = 0;

    // Create slam_state vectors. slam state vector includes robot x,y.theta and x,y for obstacles
    slam_state = arma::vec(max_obs * 2 + 3, arma::fill::zeros);
    slam_state_prev = arma::vec(max_obs * 2 + 3, arma::fill::zeros);
    obstacle_initialized = std::vector<bool>(max_obs, false);
    Q = arma::mat(3, 3, arma::fill::zeros);
    Q.diag() += 1e-3;
    R = arma::mat(2 * max_obs, 2 * max_obs, arma::fill::zeros);
    R.diag() += 1e-1;
    Q_bar = arma::mat(max_obs * 2 + 3, max_obs * 2 + 3, arma::fill::zeros);
    Q_bar.submat(0, 0, 2, 2) = Q;

    // Initialize covariance
    Covariance = arma::mat(max_obs * 2 + 3, max_obs * 2 + 3, arma::fill::zeros);
    for (int i = 0; i < 2 * max_obs; i++) {
      Covariance.at(i + 3, i + 3) = 999999;   // Set covariance high b/c nothing is known yet
    }

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);
    slam_obstacle_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/slam_obstacles",
      10);
    path_pub = create_publisher<nav_msgs::msg::Path>("green/path", 10);

    // Subscribers
    fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
      "red/fake_sensor",
      10, std::bind(&Nuslam::do_slam_callback, this, std::placeholders::_1));
    detected_obs_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
      "detected_obs",
      10, std::bind(&Nuslam::do_slam_callback, this, std::placeholders::_1));
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10, std::bind(&Nuslam::joint_states_callback, this, std::placeholders::_1));

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    path_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Nuslam::update_path, this));

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, max_obs, num_obs;
  std::string body_id, odom_id, wheel_left, wheel_right;
  double track_width, wheel_diameter, obstacle_radius;
  bool use_lidar;
  nav_msgs::msg::Path slam_path;
  geometry_msgs::msg::PoseStamped slam_pose;
  std::vector<bool> obstacle_initialized;
  arma::mat Covariance, Q, R, Q_bar;
  arma::vec slam_state, slam_state_prev;
  double left_wheel_angle, right_wheel_angle;
  turtlelib::Transform2D tf_odom_body, tf_map_odom, tf_map_body;
  turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(wheel_diameter, track_width);
  int path_counter;
  double obs_dist_thresh;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::TimerBase::SharedPtr path_timer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_obstacle_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detected_obs_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  /// \brief Callback for receiving obstacle position messages from the fake sensor
  void do_slam_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // Run prediction step
    predict_ekf(msg);

    // Publish slam tf
    publish_slam_tf();

    // Update slam marker positions
    publish_slam_obstacles();

    // Update slam path after 5 slam updates
    if (path_counter >= 5) {
      update_path();
      path_counter = 0;
    }
    path_counter++;

    // Save current slam state as prev
    slam_state_prev = slam_state;
  }

  /// \brief Publishes map to odom transform from based on slam prediction
  void publish_slam_tf()
  {
    // Get odom state and make tf
    std::vector<double> odom_state = diff_drive.return_state();
    tf_odom_body = turtlelib::Transform2D{{odom_state[0], odom_state[1]}, odom_state[2]};

    // Get tf_map_odom
    tf_map_body =
      turtlelib::Transform2D{turtlelib::Vector2D{slam_state.at(1), slam_state.at(2)}, slam_state.at(
        0)};
    tf_map_odom = tf_map_body * tf_odom_body.inv();

    // Create tf message
    geometry_msgs::msg::TransformStamped tf_map_odom_msg;
    tf_map_odom_msg.header.stamp = get_clock()->now();
    tf_map_odom_msg.header.frame_id = "map";
    tf_map_odom_msg.child_frame_id = "odom";

    // Fill out tf message
    tf_map_odom_msg.transform.translation.x = tf_map_odom.translation().x;
    tf_map_odom_msg.transform.translation.y = tf_map_odom.translation().y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, normalize_angle(tf_map_odom.rotation()));
    tf_map_odom_msg.transform.rotation.x = quaternion.x();
    tf_map_odom_msg.transform.rotation.y = quaternion.y();
    tf_map_odom_msg.transform.rotation.z = quaternion.z();
    tf_map_odom_msg.transform.rotation.w = quaternion.w();

    // Send tf_map_odom
    tf_broadcaster->sendTransform(tf_map_odom_msg);
  }

  /// \brief Callback for joint states messages
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    turtlelib::Transform2D body_tf;
    turtlelib::Twist2D body_twist;
    turtlelib::Vector2D position;
    tf2::Quaternion quaternion;
    std::vector<double> state;

    // Extract new wheel data from message
    left_wheel_angle = msg.position[0];
    right_wheel_angle = msg.position[1];

    body_twist = diff_drive.get_twist(left_wheel_angle, right_wheel_angle);
    body_tf = diff_drive.update_state(left_wheel_angle, right_wheel_angle);
    position = body_tf.translation();
    quaternion.setRPY(0, 0, body_tf.rotation());

    // RCLCPP_INFO(this->get_logger(), "left_wheel_angle: %f, right_wheel_angle: %f", left_wheel_angle, right_wheel_angle);

    // Create odom message
    nav_msgs::msg::Odometry odom_msg;
    // Refer to Citation [3] ChatGPT
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;

    // Add pose to message
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();
    odom_msg.pose.pose.position.x = position.x;
    odom_msg.pose.pose.position.y = position.y;

    // Add Twist to message
    odom_msg.twist.twist.linear.x = body_twist.x;
    odom_msg.twist.twist.linear.y = body_twist.y;
    odom_msg.twist.twist.angular.z = body_twist.omega;

    // Publish odom_msg
    odom_pub->publish(odom_msg);

    // Get state
    state = diff_drive.return_state();

    // Publish transform from odom to base_link (same position)
    geometry_msgs::msg::TransformStamped tf;
    // Refer to Citation [3] ChatGPT
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = odom_id;
    tf.child_frame_id = body_id;
    tf.transform.translation.x = state[0];
    tf.transform.translation.y = state[1];
    tf.transform.translation.z = 0.0;

    quaternion.setRPY(0, 0, state[2]);
    tf.transform.rotation.x = quaternion.x();
    tf.transform.rotation.y = quaternion.y();
    tf.transform.rotation.z = quaternion.z();
    tf.transform.rotation.w = quaternion.w();

    tf_broadcaster->sendTransform(tf);
  }

  /// \brief Extended Kalman filter prediction step
  void predict_ekf(const visualization_msgs::msg::MarkerArray & msg)
  {
    double delta_x, delta_y;
    double marker_x, marker_y, rad, phi;
    double dx_hat, dy_hat, d_hat, rad_hat, phi_hat;
    arma::mat H_j(2, 3 + 2 * max_obs, arma::fill::zeros);
    arma::vec z_k_hat(2, arma::fill::zeros);

    // Get odom state
    std::vector<double> odom_state = diff_drive.return_state();

    // Make a tf for the robot in the odom frame
    tf_odom_body = turtlelib::Transform2D{{odom_state[0], odom_state[1]}, odom_state[2]};

    // get the robot position in the map frame
    tf_map_body = tf_map_odom * tf_odom_body;
    slam_state.at(0) = normalize_angle(tf_map_body.rotation());
    slam_state.at(1) = tf_map_body.translation().x;
    slam_state.at(2) = tf_map_body.translation().y;

    // Calculate change in x and y positions
    delta_x = slam_state.at(1) - slam_state_prev.at(1);
    delta_y = slam_state.at(2) - slam_state_prev.at(2);

    // Create uncertainties matrix
    arma::mat A_t(3 + 2 * max_obs, 3 + 2 * max_obs, arma::fill::zeros);
    A_t.at(1, 0) = -delta_y;
    A_t.at(2, 0) = delta_x;

    // Add identity matrix
    A_t.diag() += 1.0;

    // Update Covariance matrix with A_t
    Covariance = (A_t * Covariance * A_t.t()) + Q_bar;

    // Add marker positions to measurement
    for (size_t i = 0; i < msg.markers.size(); i++) {
      // extract coordinates, id, and action from message
      marker_x = msg.markers.at(i).pose.position.x;
      marker_y = msg.markers.at(i).pose.position.y;
      int id = associate_obs(marker_x, marker_y); //msg.markers.at(i).id;
      // RCLCPP_INFO(this->get_logger(), "id: %d", id);
      if (id == -1) {
        // Obstacle is out of range
        continue;
      } else {
        // Convert current x and y to polar coordinates
        rad = sqrt(pow(marker_x, 2) + pow(marker_y, 2));
        phi = atan2(marker_y, marker_x);

        // Create vector with radius and angle
        arma::vec z_j(2, arma::fill::zeros);
        z_j.at(0) = rad;
        z_j.at(1) = phi;

        // check if this obstacle has been initialized
        if (!obstacle_initialized.at(id)) {
          RCLCPP_INFO(this->get_logger(), "initializing: %d", id);
          // initialize x coordinate
          slam_state.at((2 * id) + 3) = slam_state.at(1) + rad *
            cos(normalize_angle(phi + slam_state.at(0)));
          // initialize y coordinate
          slam_state.at((2 * id) + 4) = slam_state.at(2) + rad *
            sin(normalize_angle(phi + slam_state.at(0)));
          // Mark obstacle as initialized
          obstacle_initialized.at(id) = true;
        }

        // Find estimated state Z_bar
        dx_hat = slam_state.at(3 + (2 * id)) - slam_state.at(1);
        dy_hat = slam_state.at(3 + (2 * id) + 1) - slam_state.at(2);
        d_hat = dx_hat * dx_hat + dy_hat * dy_hat;
        rad_hat = sqrt(d_hat);
        phi_hat = normalize_angle(atan2(dy_hat, dx_hat) - slam_state.at(0));
        z_k_hat.at(0) = rad_hat;
        z_k_hat.at(1) = phi_hat;

        // Create H_j from predicted state
        H_j.at(1, 0) = -1.0;
        H_j.at(0, 1) = -dx_hat / rad_hat;
        H_j.at(1, 1) = dy_hat / d_hat;
        H_j.at(0, 2) = -dy_hat / rad_hat;
        H_j.at(1, 2) = -dx_hat / d_hat;
        H_j.at(0, 3 + (2 * id)) = dx_hat / rad_hat;
        H_j.at(1, 3 + (2 * id)) = -dy_hat / d_hat;
        H_j.at(0, 4 + (2 * id)) = dy_hat / rad_hat;
        H_j.at(1, 4 + (2 * id)) = dx_hat / d_hat;

        // Find kalman gain K_i
        arma::mat R_i = R.submat((2 * id), (2 * id), (2 * id) + 1, (2 * id) + 1);
        arma::mat K_i_sub = H_j * Covariance * H_j.t() + R_i;
        arma::mat K_i = Covariance * H_j.t() * K_i_sub.i();

        // Update state
        arma::vec dz_j = z_j - z_k_hat;
        dz_j.at(1) = normalize_angle(dz_j.at(1));

        slam_state = slam_state + K_i * dz_j;
        slam_state.at(0) = normalize_angle(slam_state.at(0));

        // Update covariance based on marker
        arma::mat identity = arma::eye((max_obs * 2) + 3, (max_obs * 2) + 3);
        Covariance = (identity - K_i * H_j) * Covariance;

      }
    }

  }

  /// @brief Finds index of closest obstacle to a new obstacle
  /// @param marker_x new obstacle x
  /// @param marker_y new obstacle y
  /// @return Index of closest obstacle
  int associate_obs(double marker_x, double marker_y)
  {
    const double rad = sqrt(pow(marker_x, 2) + pow(marker_y, 2));
    const double phi = normalize_angle(atan2(marker_y, marker_x));
    std::vector<double> dist_vect;
    bool new_obs = false;

    // Find maker position in map frame
    const double marker_x_map = slam_state.at(1) + rad *
      cos(normalize_angle(phi + slam_state.at(0)));
    const double marker_y_map = slam_state.at(2) + rad *
      sin(normalize_angle(phi + slam_state.at(0)));

    // Do not create new obstacles if max obstacles created
    if (num_obs != max_obs) {
      // Add current marker to state variable as if it was a new obstacle and increase num_obs
      slam_state.at(3 + (2 * num_obs)) = marker_x_map;
      slam_state.at(4 + (2 * num_obs)) = marker_y_map;
      num_obs += 1;
      new_obs = true;
    }

    // Iterate through obstacle measurementss
    for (int i = 0; i < num_obs; i++) {

      // Find Euclidian distance (works well enough here)
      const double x_obs_est = slam_state.at(3 + (2 * i));
      const double y_obs_est = slam_state.at(4 + (2 * i));
      const double dist =
        sqrt(pow((x_obs_est - marker_x_map), 2) + pow((y_obs_est - marker_y_map), 2));

      // Add to dist_vect vector
      dist_vect.push_back(dist);
    }

    // Set distance for new obstacle to be 0.25 m if new obs
    if (new_obs) {
      dist_vect.at(num_obs - 1) = obs_dist_thresh;
    }


    // Find minimum distance and get it's index
    auto d_star = dist_vect.at(0);
    int index = 0;
    for (int i = 0; i < static_cast<int>(dist_vect.size()); i++) {
      if (dist_vect.at(i) < d_star) {
        d_star = dist_vect.at(i);
        index = i;
      }
    }

    // Check if index matches added obstacle, if so this is a new obstacle
    if ((index != (num_obs - 1)) && (num_obs != max_obs)) {
      num_obs += -1;
      // clear out the obstacle
      slam_state.at(3 + (2 * num_obs)) = 0.;
      slam_state.at(4 + (2 * num_obs)) = 0.;
    }

    return index;
  }

  /// \brief Normalizes angles to range -pi to pi
  double normalize_angle(double angle)
  {
    while (angle > 3.14159265) {
      angle += -2 * 3.14159265;
    }
    while (angle < -3.14159265) {
      angle += 2 * 3.14159265;
    }

    return angle;
  }

  /// \brief Updates the path of the slam robot with the current pose and publishes the path
  void update_path()
  {
    // Update ground truth red turtle path
    slam_path.header.stamp = get_clock()->now();
    slam_path.header.frame_id = "nusim/world";

    // Create new pose stamped
    slam_pose.header.stamp = get_clock()->now();
    slam_pose.header.frame_id = "map";
    slam_pose.pose.position.x = slam_state.at(1);
    slam_pose.pose.position.y = slam_state.at(2);
    slam_pose.pose.position.z = 0.0;

    // Add rotation quaternion about Z
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, slam_state.at(0));
    slam_pose.pose.orientation.x = quaternion.x();
    slam_pose.pose.orientation.y = quaternion.y();
    slam_pose.pose.orientation.z = quaternion.z();
    slam_pose.pose.orientation.w = quaternion.w();

    // Add pose to path and publish path
    slam_path.poses.push_back(slam_pose);
    path_pub->publish(slam_path);
  }

  /// \brief Publishes position estimate of obstacles according to slam
  void publish_slam_obstacles()
  {
    visualization_msgs::msg::MarkerArray obstacles;

    for (int i = 0; i < max_obs; i++) {
      // Create obstacle marker
      visualization_msgs::msg::Marker obstacle;
      obstacle.header.stamp = get_clock()->now();
      obstacle.header.frame_id = "map";
      obstacle.id = i;
      obstacle.type = 3;  // cylinder
      obstacle.action = 0;

      // Set color
      obstacle.color.r = 0.0;
      obstacle.color.g = 1.0;
      obstacle.color.b = 0.0;
      obstacle.color.a = 1.0;

      // Set radius and height
      obstacle.scale.x = 2 * obstacle_radius;
      obstacle.scale.y = 2 * obstacle_radius;
      obstacle.scale.z = 0.25;

      // Set obstacle location
      obstacle.pose.position.x = slam_state.at((2 * i) + 3);
      obstacle.pose.position.y = slam_state.at((2 * i) + 4);
      obstacle.pose.position.z = 0.125;

      // Add to marker array if real obstacle
      if ((obstacle.pose.position.x != 0.) && (obstacle.pose.position.y != 0.)) {
        obstacles.markers.push_back(obstacle);
      }
    }

    // Publish obstacle markers
    slam_obstacle_pub->publish(obstacles);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nuslam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
