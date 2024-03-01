/// \file nuslam.cpp
/// \brief SLAM for the turtlebot
///
/// PARAMETERS:
///     rate (double): frequency of the timer (Hz)
///     x0 (double): initial x position of the turtlebot (m)
///     y0 (double): initial y position of the turtlebot (m)
///     theta0 (double): initial theta angle of the turtlebot (rad)
///     obstacles_x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles_y (double[]): list of y coordinates of cylindrical obstacles (m)
///     obstacles_radius (double): radius of all cylindrical obstacles (m)
///     arena_x_length (double): X length of rectangular arena (m)
///     arena_y_length (double): Y length of rectangular arena (m)
///     input_noise (double): the noise variance for moving the robot
///     slip_fraction (double): defines how much the wheels slip
///     max_range (double): maximum simulated lidar range
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

// Used ChatGPT for debugging
// Refer to Citation [5] ChatGPT
// Refer to Citation [6] ChatGPT

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
    declare_parameter("rate", 200);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_int();

    // Create diff_drive, initialize wheel speeds
    diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
    right_wheel_vel = 0.;
    left_wheel_vel = 0.;

    // Create noise and slip ranges
    noise_range = std::normal_distribution<>{0, sqrt(input_noise)};
    slip_range = std::uniform_real_distribution<>{-1.0 * slip_fraction, slip_fraction};
    sensor_range = std::normal_distribution<>{0, sqrt(basic_sensor_variance)};
    laser_range = std::normal_distribution<>{0, sqrt(laser_noise)};

    // Publishers
    timestep_publisher = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacle_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    walls_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    fake_sensor_pub = create_publisher<visualization_msgs::msg::MarkerArray>("red/fake_sensor", 10);
    laser_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // Subscribers
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd",
      10, std::bind(&Nuslam::wheel_cmd_callback, this, std::placeholders::_1));

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Services
    reset_server = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nuslam::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Nuslam::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int timestep = 0;
  double x0, y0, theta0;
  double arena_x_length, arena_y_length;
  double wheel_radius, track_width;
  double right_wheel_vel, left_wheel_vel, motor_cmd_per_rad_sec;
  turtlelib::DiffDrive diff_drive = turtlelib::DiffDrive(wheel_radius, track_width);
  std::vector<double> obstacles_x, obstacles_y;
  double obstacles_radius;
  double x_gt, y_gt, theta_gt;
  int encoder_ticks_per_rad, laser_samples;
  int loop_rate, pose_rate, fake_sensor_rate;
  double slip_fraction, input_noise, basic_sensor_variance, laser_noise;
  double max_range, min_range;
  double collision_radius;
  nav_msgs::msg::Path nuslam_path;
  geometry_msgs::msg::PoseStamped nuslam_pose;
  std::normal_distribution<> noise_range, sensor_range, laser_range;
  std::uniform_real_distribution<> slip_range;
  bool draw_only;

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nuslam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
