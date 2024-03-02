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
///     red/path (nav_msgs::msg::Path): the path of the nusim robot
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
    declare_parameter("odom", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("max_obstacles", 5);
    declare_parameter("obstacle_radius", 0.038);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();
    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    max_obstacles = get_parameter("max_obstacles").as_int();
    obstacle_radius = get_parameter("obstacle_radius").as_double();

    // Check if parameters have been defined. if not, throw runtime error
    // Refer to Citation [2] ChatGPT
    if (wheel_radius == -1.0 || track_width == -1.0) {
      throw std::runtime_error("Diff drive parameters not defined.");
    }
    if (body_id == "" || wheel_left == "" || wheel_right == "") {
      throw std::runtime_error("Frame parameters not defined.");
    }

    // Create diff_drive, initialize wheel speeds
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
    right_wheel_vel = 0.;
    left_wheel_vel = 0.;

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    slam_obstacle_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/slam_obstacles", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);

    // Subscribers
    fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
      "red/fake_sensor",
      10, std::bind(&Nuslam::fake_sensor_callback, this, std::placeholders::_1));
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10, std::bind(&Nuslam::joint_states_callback, this, std::placeholders::_1));

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    path_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Nusim::update_path, this));

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, max_obstacles;
  std::string body_id, odom_id, wheel_left, wheel_right;
  double track_width, wheel_radius, obstacle_radius;
  bool use_lidar;
  nav_msgs::msg::Path slam_path;
  geometry_msgs::msg::PoseStamped slam_pose;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::TimerBase::SharedPtr path_timer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_obstacle_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Subscriber<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
  rclcpp::Subscriber<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  /// @brief Callback for receiving obstacle position messages from the fake sensor
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & sensor)
  {

  }

  /// @brief Callback for joint states messages
  void joint_states_callback(const visualization_msgs::msg::MarkerArray & sensor)
  {

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
    slam_pose.pose.position.x = ;
    slam_pose.pose.position.y = ;
    slam_pose.pose.position.z = 0.0;

    // Add rotation quaternion about Z
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, );
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
